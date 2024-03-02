package frc.robot;

import frc.robot.util.TunableNumber;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;

// TODO(user): Tune all TunableNumbers for better robot control
/** Global constants. Should not include functional code. */
public class Constants {
  /** Non-functional information for software testing and metadata. */
  public static class Config {
    /** {@code true} to stream log file data to NetworkTables (takes up bandwith and processing time, but useful for concurrent running and visualization) */
    public static final boolean NTStream = false;
    /** Directory for log file. Leave blank to store in roboRIO or RIO-attached USB. */
    public static final String DATA_LOG_DIR = "";
  }

  /** Ports and properties of non-motor devices. */
  public static class DeviceConsts {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  /** Physical parts of the robot, such as gearboxes or wheel diameters. */
  public static class PhysConsts {
    public static final int kNEOMaxVoltage = 12; // NEO V1.1 nominal voltage
    public static final int kNEOCurrentLimit = 40; // NEO V1.1 general current limit (40A-60A is advised)

    public static final double kSwerveDriveRPS = 5680d / 60d; // NEO V1.1 empirical free speed
    public static final double kSwerveDriveGearbox = 1d/5.355; // SDS L3 modules with 16T drive pinion
    public static final double kSwerveWheelCircumferenceMeters = 0.099 * Math.PI; // Billet wheels
  }

  // TODO(user): Check all drivetrain measurements and limits and ensure accuracy
  /** Data relating to the entire drivetrain. */
  public static class DriveConsts {
    // Upper bound drivetrain constraints
    public static final double kMaxLinearVelMetersPerSecond = PhysConsts.kSwerveDriveRPS * PhysConsts.kSwerveDriveGearbox * PhysConsts.kSwerveWheelCircumferenceMeters * 0.75; // 75% of theoretical max (motor RPS * gearbox * wheel circumfrence * 80%)
    public static final double kMaxTurnVelRadiansPerSecond = kMaxLinearVelMetersPerSecond / Constants.SwerveConsts.kSwerve_fl.location.getDistance(new Translation2d()); // ω = velocity / radius (assuming square swerve drivetrain)
    public static final double kModuleAzimuthMaxVoltage = 0.65 * PhysConsts.kNEOMaxVoltage; // Maximum azimuth motor voltage
    public static final double kModuleDriveMaxVoltage = 0.9 * PhysConsts.kNEOMaxVoltage; // Maximum drive motor voltage (to avoid brownouts)
    public static final int kModuleAzimuthCurrentLimit = 30;

    // Multipliers for all teleop driving
    public static final double kTeleopSpeedMult = 1;
    public static final double kTeleopTurnMult = 9.5 / kMaxTurnVelRadiansPerSecond; // Set maximum teleop turn speed to 1.5 rotations/s
  }

  /** Data for each individual swerve module. */
  public static class SwerveConsts {
    // Gains for module velocity error -> voltage
    public static final TunableNumber
      kDriveS = new TunableNumber("S", 0.1, "Module Drive"),
      kDriveP = new TunableNumber("P", 0.015, "Module Drive");

    // Whether azimuth motor is inverted, use for mk4i's
    public static final boolean kAzimuthInverted = true;

    public static final SwerveModuleConstants
      kSwerve_fl = new SwerveModuleConstants(
        "SwerveFL",
        0.1, 0.095, 0.0006, // Gains
        -0.100341, // Offset
        11, 12, 13,
        new Translation2d(0.14605, 0.24765)
      ),
      kSwerve_fr = new SwerveModuleConstants(
        "SwerveFR",
        0.095, 0.1, 0.00065, // Gains
        -0.673096, // Offset
        21, 22, 23,
        new Translation2d(0.14605, -0.24765)
      ),
      kSwerve_bl = new SwerveModuleConstants(
        "SwerveBL",
        0.08, 0.105, 0.0004, // Gains
        -0.086670, // Offset
        31, 32, 33,
        new Translation2d(-0.24765, 0.24765)
      ),
      kSwerve_br = new SwerveModuleConstants(
        "SwerveBR",
        0.092, 0.09, 0.00065, // Gains
        -0.687012, // Offset
        41, 42, 43,
        new Translation2d(-0.24765, -0.24765)
      );
  }

  public static class AutoConsts {
    // TODO(dev/user): Ensure that drivetrain acceleration limits are strong
    // Upper bound drivetrain accelerations for path following and pose targeting
    public static final double kMaxLinearAccelMetersPerSecondSquared = DriveConsts.kMaxLinearVelMetersPerSecond  / 0.5; // Reaches max speed in 0.5 seconds
    public static final double kMaxTurnAccelRadiansPerSecondSquared = DriveConsts.kMaxTurnVelRadiansPerSecond / 0.5; // Reaches max speed in 0.5 seconds

    // Gains for drivetrain position error -> velocity
    public static final TunableNumber
      kTranslateP = new TunableNumber("P", 1, "Robot Translation"),
      kTranslateI = new TunableNumber("I", 0, "Robot Translation"),
      kTranslateD = new TunableNumber("D", 0, "Robot Translation");
    public static final TunableNumber
      kRotateP = new TunableNumber("P", 1, "Robot Rotation"),
      kRotateI = new TunableNumber("I", 0, "Robot Rotation"),
      kRotateD = new TunableNumber("D", 0, "Robot Rotation");
  }
}