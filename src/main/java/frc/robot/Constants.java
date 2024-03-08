package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;
import frc.robot.util.TunableNumber;

/** Global constants. Should not include functional code. */
public class Constants {
  /** Information for testing and robot configuration that must be updated consistenly. */
  public static class Config {
    /**
     * Directory for log file. Blank to store in default directory on roboRIO or RIO-attached USB.
     */
    public static final String DATA_LOG_DIR = "";

    /**
     * Set true to stream log file data to NetworkTables (takes up bandwith and processing time, but
     * useful for concurrent running and visualization)
     */
    public static final boolean NTStream = false;

    // Pigeon mount offsets - REMEMBER TO UPDATE, or configure through PhoenixTuner
    public static final MountPoseConfigs kPigeonMountPose =
        new MountPoseConfigs().withMountPoseYaw(0).withMountPosePitch(0).withMountPoseRoll(0);
  }

  /** Ports and properties of non-motor devices. */
  public static class DeviceConsts {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  /** Physical parts of the robot, such as gearboxes or wheel diameters. */
  public static class PhysConsts {
    /** NEO V1.1 nominal voltage */
    public static final int kNEOMaxVoltage = 12;

    /** NEO V1.1 general current limit (40A-60A is advised) */
    public static final int kNEOCurrentLimit = 40;

    /** NEO V1.1 empirical free speed */
    public static final double kSwerveDriveRPS = 5680d / 60d;

    /** SDS L3 modules with 16T drive pinion */
    public static final double kSwerveDriveGearbox = 1d / 5.355;

    /** Billet wheels */
    public static final double kSwerveWheelCircumferenceMeters = 0.099 * Math.PI;
  }

  /** Data relating to the entire drivetrain. */
  public static class DriveConsts {
    // Upper bound drivetrain constraints
    /** 75% of theoretical max */
    public static final double kMaxLinearVelMetersPerSecond =
        PhysConsts.kSwerveDriveRPS
            * PhysConsts.kSwerveDriveGearbox
            * PhysConsts.kSwerveWheelCircumferenceMeters
            * 0.75;

    // ω = velocity / radius
    public static final double kMaxTurnVelRadiansPerSecond =
        kMaxLinearVelMetersPerSecond / Constants.SwerveConsts.kDriveBaseRadius;

    // To avoid brownouts and overpowering
    public static final double kModuleAzimuthMaxVoltage = 0.65 * PhysConsts.kNEOMaxVoltage;
    public static final double kModuleDriveMaxVoltage = 0.95 * PhysConsts.kNEOMaxVoltage;
    public static final int kModuleAzimuthCurrentLimit = 30;

    // Multipliers for all teleop driving
    public static final double kTeleopSpeedMult = 1;
    // Set maximum teleop turn speed to 1.5 rotations/s
    public static final double kTeleopTurnMult = 9.5 / kMaxTurnVelRadiansPerSecond;

    /** Update rate for drivetrain, default 20 ms. 8-64 ms is best for NEO hall sensor. */
    public static final int kPeriodMs = 10;
  }

  public static class ShooterConsts {
    public static final byte kTopShooterMotorID = 5;
    public static final byte kBottomShooterMotorID = 4;

    public static final double kSourceIntakeVolts = 9;
    public static final double kShootVolts = -6;
    public static final double kSpitVolts = -3;
  }

  public static class FeederConsts {
    public static final byte kFeedMotorID = 3;

    public static final double kFeedVolts = -3;
    public static final double kDeepIntakeVolts = 1;
  }

  /** Data for each individual swerve module. */
  public static class SwerveConsts {
    // Gains for module velocity error -> voltage
    public static final TunableNumber kDriveS = new TunableNumber("S", 0.1, "Module Drive");
    public static final TunableNumber kDriveP = new TunableNumber("P", 2, "Module Drive");

    // Whether azimuth motor is inverted, use for mk4i's
    public static final boolean kAzimuthInverted = true;

    // TODO: final swerve offset tuning
    public static final SwerveModuleConstants
        kSwerve_fl =
            new SwerveModuleConstants(
                "SwerveFL",
                // Azimuth gains (kS, kP, kD)
                0.1,
                0.095,
                0.0006,
                // CANcoder offset
                -0.100341,
                11,
                12,
                13,
                new Translation2d(0.14605, 0.24765)),
        kSwerve_fr =
            new SwerveModuleConstants(
                "SwerveFR",
                // Azimuth gains (kS, kP, kD)
                0.092,
                0.1,
                0.00065,
                // CANcoder offset
                -0.673096,
                21,
                22,
                23,
                new Translation2d(0.14605, -0.24765)),
        kSwerve_bl =
            new SwerveModuleConstants(
                "SwerveBL",
                // Azimuth gains (kS, kP, kD)
                0.08,
                0.105,
                0.0004,
                // CANcoder offset
                -0.086670,
                31,
                32,
                33,
                new Translation2d(-0.24765, 0.24765)),
        kSwerve_br =
            new SwerveModuleConstants(
                "SwerveBR",
                // Azimuth gains (kS, kP, kD)
                0.092,
                0.09,
                0.00065,
                // CANcoder offset
                -0.687012,
                41,
                42,
                43,
                new Translation2d(-0.24765, -0.24765));

    /** Drive base radius for angularvelocity calcs (use swerve module farthest from COR) */
    public static final double kDriveBaseRadius =
        kSwerve_bl.location.getDistance(new Translation2d());
  }

  // TODO(user): Tune pathfinding PID gains
  public static class AutoConsts {
    // TODO(dev/user): Ensure that drivetrain acceleration limits are strong
    // Upper bound drivetrain accelerations for path following and pose targeting
    public static final double kMaxLinearAccelMetersPerSecondSquared =
        DriveConsts.kMaxLinearVelMetersPerSecond;
    public static final double kMaxTurnAccelRadiansPerSecondSquared =
        DriveConsts.kMaxTurnVelRadiansPerSecond;

    // Gains for drivetrain position error -> velocity
    public static final TunableNumber kTranslateP = new TunableNumber("P", 5, "Robot Translation");
    public static final TunableNumber kTranslateD = new TunableNumber("D", 0, "Robot Translation");
  }
}
