package frc.robot.logger;

import frc.robot.Constants.Config;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.HashMap;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotBase;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * <p> Huge thanks to FRC Team 581: Littleton Robotics for the basis for this implementation. </p>
 *
 * <p> For consistency, logging calls should be done by subsystems and the main robot class (after control logic), and be under a directory corresponding to the same. </p>
 *
 * A logger that is similar to the AdvantageKit `Logger` class. It uses WPILib's DataLog to log the data to a file, and streams the data (if needed) using NetworkTables 4.
 *
 * @see https://github.com/team581/frc-2023-charged-up/pull/64
 */
public class Logger {
  private static final String OUTPUT_LOG_DIR = "/RealOutputs/";
  private static final String METADATA_LOG_DIR = "/RealMetadata/";

  /** List of loggables to be updated each period. */
  private static final ArrayList<Loggable> loggables = new ArrayList<>();

  /** Map to turn keys into integers for better performance on other maps. */
  private static final HashMap<String, Integer> keyToID = new HashMap<>();

  // Entry maps for logging to disk
  private static final HashMap<Integer, BooleanLogEntry> booleanLogs = new HashMap<>();
  private static final HashMap<Integer, DoubleLogEntry> doubleLogs = new HashMap<>();
  private static final HashMap<Integer, FloatLogEntry> floatLogs = new HashMap<>();
  private static final HashMap<Integer, IntegerLogEntry> integerLogs = new HashMap<>();
  private static final HashMap<Integer, StringLogEntry> stringLogs = new HashMap<>();
  private static final HashMap<Integer, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private static final HashMap<Integer, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private static final HashMap<Integer, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private static final HashMap<Integer, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private static final HashMap<Integer, StringArrayLogEntry> stringArrayLogs = new HashMap<>();

  // Network tables Tables for data logging directories
  private static NetworkTable outputTable;
  private static NetworkTable metadataTable;
  /** PubSubOptions to use for logging. */
  private static final PubSubOption pubOptions = PubSubOption.keepDuplicates(true);

  // Publisher maps for logging to network tables
  private static final HashMap<Integer, BooleanPublisher> booleanPublishers = new HashMap<>();
  private static final HashMap<Integer, DoublePublisher> doublePublishers = new HashMap<>();
  private static final HashMap<Integer, FloatPublisher> floatPublishers = new HashMap<>();
  private static final HashMap<Integer, IntegerPublisher> integerPublishers = new HashMap<>();
  private static final HashMap<Integer, StringPublisher> stringPublishers = new HashMap<>();
  private static final HashMap<Integer, BooleanArrayPublisher> booleanArrayPublishers = new HashMap<>();
  private static final HashMap<Integer, DoubleArrayPublisher> doubleArrayPublishers = new HashMap<>();
  private static final HashMap<Integer, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private static final HashMap<Integer, IntegerArrayPublisher> integerArrayPublishers = new HashMap<>();
  private static final HashMap<Integer, StringArrayPublisher> stringArrayPublishers = new HashMap<>();

  // Cannot be initialized until after start to ensure proper file creation
  private static DataLog log;

  // Start log manager
  static {
    DataLogManager.start(
      RobotBase.isSimulation() ? "" : Config.DATA_LOG_DIR, // Log to project directory in simulation
      "FRC_" + LocalDateTime.now(ZoneId.of("UTC-8")).format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH.mm.ss")) + ".wpilog"); // File name

    log = DataLogManager.getLog();
    // Disable logging NT values to separate logger and networktables
    DataLogManager.logNetworkTables(false);
    // Log inputs from DriverStation
    DriverStation.startDataLog(log);

    // Set up network tables for data stream
    if (Config.NTStream) {
      var table = NetworkTableInstance.getDefault().getTable("AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }
  }

  /** @param loggable loggable to register for periodic update calls */
  public static void registerLoggable(Loggable loggable) {
    loggables.add(loggable);
    log("Device " + loggable.getDirectory() + " initialized");
  }

  /** Update all registered loggables. Should be called every period. */
  public static void updateLogs() {
    loggables.forEach(e -> e.log());
  }

  /**
   * Log a message to the "messages" entry. The message is also printed to standard output.
   *
   * @param msg message
   */
  public static void log(String msg) {
    DataLogManager.log(msg);
  }

  /** Returns the ID of a given key, or creates it if not present. Hopefully will remove performance issues. */
  public static int getID(String key) {
    return keyToID.computeIfAbsent(key, k -> keyToID.size());
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, boolean value) {
    int id = getID(path);

    booleanLogs
      .computeIfAbsent(id, k -> new BooleanLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      booleanPublishers
      .computeIfAbsent(id, k -> outputTable.getBooleanTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, double value) {
    int id = getID(path);

    doubleLogs
      .computeIfAbsent(id, k -> new DoubleLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      doublePublishers
        .computeIfAbsent(id, k -> outputTable.getDoubleTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, float value) {
    int id = getID(path);

    floatLogs
      .computeIfAbsent(id, k -> new FloatLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      floatPublishers
        .computeIfAbsent(id, k -> outputTable.getFloatTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, int value) {
    int id = getID(path);

    integerLogs
      .computeIfAbsent(id, k -> new IntegerLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      integerPublishers
        .computeIfAbsent(id, k -> outputTable.getIntegerTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, String value) {
    int id = getID(path);

    stringLogs
      .computeIfAbsent(id, k -> new StringLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      stringPublishers
        .computeIfAbsent(id, k -> outputTable.getStringTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, boolean[] values) {
    int id = getID(path);

    booleanArrayLogs
      .computeIfAbsent(id, k -> new BooleanArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      booleanArrayPublishers
        .computeIfAbsent(id, k -> outputTable.getBooleanArrayTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, double[] values) {
    int id = getID(path);

    doubleArrayLogs
      .computeIfAbsent(id, k -> new DoubleArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      doubleArrayPublishers
        .computeIfAbsent(id, k -> outputTable.getDoubleArrayTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, float[] values) {
    int id = getID(path);

    floatArrayLogs
      .computeIfAbsent(id, k -> new FloatArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      floatArrayPublishers
        .computeIfAbsent(id, k -> outputTable.getFloatArrayTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, long[] values) {
    int id = getID(path);

    integerArrayLogs
      .computeIfAbsent(id, k -> new IntegerArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      integerArrayPublishers
        .computeIfAbsent(id, k -> outputTable.getIntegerArrayTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, int[] values) {
    // Transform int[] into long[]
    long[] longs = new long[values.length];
    for (int i = 0; i < values.length; i++) {
      longs[i] = values[i];
    }

    recordOutput(path, longs);
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, String[] values) {
    int id = getID(path);

    stringArrayLogs
      .computeIfAbsent(id, k -> new StringArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      stringArrayPublishers
        .computeIfAbsent(id, k -> outputTable.getStringArrayTopic(path).publish(pubOptions)) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, Pose2d... value) {
    // Map Pose2d[] into double[]
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }

    recordOutput(path, data);
  }

  public static void recordOutput(String path, Pose3d... value) {
    // Map Pose3d[] into double[]
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }

    recordOutput(path, data);
  }

  public static void recordOutput(String path, Trajectory value) {
    // Map trajectory into Pose2d[]
    recordOutput(path,
      value.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new));
  }

  public static void recordOutput(String path, ChassisSpeeds value) {
    // Map speeds in double[]
    recordOutput(path, new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond});
  }

  public static void recordOutput(String path, SwerveModuleState... values) {
    // Map states into double[]
    double[] data = new double[values.length * 2];
    for (int i = 0; i < values.length; i++) {
      data[i * 2] = values[i].angle.getRadians(); // Angle recorded in radians
      data[i * 2 + 1] = values[i].speedMetersPerSecond;
    }

    recordOutput(path, data);
  }

  public static void recordMetadata(String path, String value) {
    // Record metadata in log file
    var entry = new StringLogEntry(log, METADATA_LOG_DIR + path);
    entry.append(value);
    entry.finish(); // Close unused entry

    // Record metadata to network tables
    if (Config.NTStream) {
      var publisher = metadataTable.getStringTopic(path).publish();
      publisher.set(value);
      publisher.close(); // Close unused publisher
    }
  }
}