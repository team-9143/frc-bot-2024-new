package frc.robot.logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.Config;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Huge thanks to FRC Team 581: Littleton Robotics for the basis for this implementation.
 *
 * <p>For consistency, logging calls should be done by subsystems and the main robot class (after
 * control logic), and be under a directory corresponding to the same. A logger that is similar to
 * the AdvantageKit `Logger` class. It uses WPILib's DataLog to log the data to a file, and streams
 * the data (if needed) using NetworkTables 4.
 *
 * @see https://github.com/team581/frc-2023-charged-up/pull/64
 */
public class Logger {
  private static final String OUTPUT_LOG_DIR = "/RealOutputs/";
  private static final String METADATA_LOG_DIR = "/RealMetadata/";

  /** List of loggables to be updated each period. */
  private static final ArrayList<Loggable> loggables = new ArrayList<>();

  // Entry maps for logging to disk
  private static final HashMap<String, BooleanLogEntry> booleanLogs = new HashMap<>();
  private static final HashMap<String, DoubleLogEntry> doubleLogs = new HashMap<>();
  private static final HashMap<String, FloatLogEntry> floatLogs = new HashMap<>();
  private static final HashMap<String, IntegerLogEntry> integerLogs = new HashMap<>();
  private static final HashMap<String, StringLogEntry> stringLogs = new HashMap<>();
  private static final HashMap<String, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private static final HashMap<String, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private static final HashMap<String, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private static final HashMap<String, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private static final HashMap<String, StringArrayLogEntry> stringArrayLogs = new HashMap<>();

  // Network tables Tables for data logging directories
  private static NetworkTable outputTable;
  private static NetworkTable metadataTable;

  /** PubSubOptions to use for logging. */
  private static final PubSubOption pubOptions = PubSubOption.keepDuplicates(true);

  // Publisher maps for logging to network tables
  private static final HashMap<String, BooleanPublisher> booleanPublishers = new HashMap<>();
  private static final HashMap<String, DoublePublisher> doublePublishers = new HashMap<>();
  private static final HashMap<String, FloatPublisher> floatPublishers = new HashMap<>();
  private static final HashMap<String, IntegerPublisher> integerPublishers = new HashMap<>();
  private static final HashMap<String, StringPublisher> stringPublishers = new HashMap<>();
  private static final HashMap<String, BooleanArrayPublisher> booleanArrayPublishers =
      new HashMap<>();
  private static final HashMap<String, DoubleArrayPublisher> doubleArrayPublishers =
      new HashMap<>();
  private static final HashMap<String, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private static final HashMap<String, IntegerArrayPublisher> integerArrayPublishers =
      new HashMap<>();
  private static final HashMap<String, StringArrayPublisher> stringArrayPublishers =
      new HashMap<>();

  // Cannot be initialized until after start to ensure proper file creation
  private static DataLog m_log;

  // Start log manager
  static {
    DataLogManager.start(
        // Log to default directory in simulation
        RobotBase.isSimulation() ? "" : Config.DATA_LOG_DIR,
        "FRC_"
            + LocalDateTime.now(ZoneId.of("UTC-8"))
                .format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH.mm.ss"))
            + ".wpilog");

    m_log = DataLogManager.getLog();
    // Disable logging NT values to separate logger and networktables
    DataLogManager.logNetworkTables(false);
    // Log inputs from DriverStation
    DriverStation.startDataLog(m_log);

    // Set up network tables for data stream
    if (Config.NTStream) {
      var table = NetworkTableInstance.getDefault().getTable("AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }
  }

  /**
   * @param loggable loggable to register for periodic update calls
   */
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

  @SuppressWarnings("resource")
  public static void recordOutput(String path, boolean value) {
    // Retrieve/create log entry and record value
    booleanLogs
        .computeIfAbsent(path, k -> new BooleanLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(value);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      booleanPublishers
          .computeIfAbsent(path, k -> outputTable.getBooleanTopic(path).publish(pubOptions))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, double value) {
    // Retrieve/create log entry and record value
    doubleLogs
        .computeIfAbsent(path, k -> new DoubleLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(value);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      doublePublishers
          .computeIfAbsent(path, k -> outputTable.getDoubleTopic(path).publish(pubOptions))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, float value) {
    // Retrieve/create log entry and record value
    floatLogs
        .computeIfAbsent(path, k -> new FloatLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(value);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      floatPublishers
          .computeIfAbsent(path, k -> outputTable.getFloatTopic(path).publish(pubOptions))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, int value) {
    // Retrieve/create log entry and record value
    integerLogs
        .computeIfAbsent(path, k -> new IntegerLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(value);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      integerPublishers
          .computeIfAbsent(path, k -> outputTable.getIntegerTopic(path).publish(pubOptions))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, String value) {
    // Retrieve/create log entry and record value
    stringLogs
        .computeIfAbsent(path, k -> new StringLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(value);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      stringPublishers
          .computeIfAbsent(path, k -> outputTable.getStringTopic(path).publish(pubOptions))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, boolean[] values) {
    // Retrieve/create log entry and record value
    booleanArrayLogs
        .computeIfAbsent(path, k -> new BooleanArrayLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(values);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      booleanArrayPublishers
          .computeIfAbsent(path, k -> outputTable.getBooleanArrayTopic(path).publish(pubOptions))
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, double[] values) {
    // Retrieve/create log entry and record value
    doubleArrayLogs
        .computeIfAbsent(path, k -> new DoubleArrayLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(values);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      doubleArrayPublishers
          .computeIfAbsent(path, k -> outputTable.getDoubleArrayTopic(path).publish(pubOptions))
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, float[] values) {
    // Retrieve/create log entry and record value
    floatArrayLogs
        .computeIfAbsent(path, k -> new FloatArrayLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(values);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      floatArrayPublishers
          .computeIfAbsent(path, k -> outputTable.getFloatArrayTopic(path).publish(pubOptions))
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String path, long[] values) {
    // Retrieve/create log entry and record value
    integerArrayLogs
        .computeIfAbsent(path, k -> new IntegerArrayLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(values);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      integerArrayPublishers
          .computeIfAbsent(path, k -> outputTable.getIntegerArrayTopic(path).publish(pubOptions))
          .set(values);
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
    // Retrieve/create log entry and record value
    stringArrayLogs
        .computeIfAbsent(path, k -> new StringArrayLogEntry(m_log, OUTPUT_LOG_DIR + path))
        .append(values);

    if (Config.NTStream) {
      // Retrieve/create publisher and record value
      stringArrayPublishers
          .computeIfAbsent(path, k -> outputTable.getStringArrayTopic(path).publish(pubOptions))
          .set(values);
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
    recordOutput(
        path, value.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new));
  }

  public static void recordOutput(String path, ChassisSpeeds value) {
    // Map speeds in double[]
    recordOutput(
        path,
        new double[] {
          value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond
        });
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
    var entry = new StringLogEntry(m_log, METADATA_LOG_DIR + path);
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
