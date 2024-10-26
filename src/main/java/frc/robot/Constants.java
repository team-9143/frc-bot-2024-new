package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;
import frc.robot.util.TunableNumber;

// Global constants. Should not include functional code.
public class Constants {
  // Information for testing and robot configuration that must be updated consistenly.
  public static class Config {
    /**
     * Set true to stream log file data to NetworkTables (takes up bandwith and processing time, but
     * useful for concurrent running and visualization).
     */
    public static final boolean NTStream = false;

    // Pigeon mount offsets - REMEMBER TO UPDATE, or configure through PhoenixTuner.
    public static final MountPoseConfigs kPigeonMountPose =
        new MountPoseConfigs().withMountPoseYaw(0).withMountPosePitch(0).withMountPoseRoll(0);
  }

  // Ports and properties of non-motor devices.
  public static class DeviceConstants {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  // Physical parts of the robot, such as gearboxes or wheel diameters.
  public static class PhysicalConstants {
    // NEO V1.1 nominal voltage.
    public static final int kNEOMaxVoltage = 12;

    // NEO V1.1 general current limit (40A-60A is advised).
    public static final int kNEOCurrentLimit = 40;

    // NEO V1.1 empirical free speed.
    public static final double kSwerveDriveMaxRPS = 5680d / 60d;

    // Ratio of drive wheels to motors for SDS L3 modules with 16T drive pinions.
    public static final double kSwerveDriveMechToSens = 1d / 5.355;

    // Billet wheels.
    public static final double kSwerveWheelCircumferenceMeters = 0.099 * Math.PI;

    // Ratio of feeder wheels to motor.
    public static final double kFeederMechToSens = 11d / 32d;

    // Ratio of shooter wheels to motors.
    public static final double kShooterMechToSens = 1;
  }

  // Data relating to the entire drivetrain.
  public static class DriveConstants {
    // Upper bound drivetrain constraints.
    // Competition speed: 80% of theoretical max.
    public static final double kMaxLinearVelMetersPerSecond =
        PhysicalConstants.kSwerveDriveMaxRPS
            * PhysicalConstants.kSwerveDriveMechToSens
            * PhysicalConstants.kSwerveWheelCircumferenceMeters
            * 0.8;

    // ω = velocity / radius.
    public static final double kMaxTurnVelRadiansPerSecond =
        kMaxLinearVelMetersPerSecond / Constants.SwerveConstants.kDriveBaseRadius;

    // To avoid brownouts and overpowering.
    public static final double kModuleAzimuthMaxVoltage = 0.65 * PhysicalConstants.kNEOMaxVoltage;
    public static final double kModuleDriveMaxVoltage = 0.95 * PhysicalConstants.kNEOMaxVoltage;
    public static final int kModuleAzimuthCurrentLimit = 40;

    // Multipliers for all teleop driving.
    public static final double kTeleopSpeedMult = 1;
    // Set maximum teleop turn speed to 1.5 rotations/s.
    public static final double kTeleopTurnMult = 9.5 / kMaxTurnVelRadiansPerSecond;

    // Update rate for drivetrain, default 20 ms. 8-64 ms is best for NEO hall sensor.
    public static final int kPeriodMs = 10;
  }

  public static class AmperConstants {
    // CAN ID for the Spark MAX motor controller.
    public static final int kAmperID = 4;

    // Current limit for the Spark MAX motor controller.
    public static final int kAmperCurrentLimit = 60;

    // Speeds for intaking and scoring. Intaking speed is negative to run the motors in reverse.
    public static final double kAmperIntakeSpeed = 0.4;
    public static final double kAmperScoreSpeed = -0.4;
    public static final double kAmperHoldPositionSpeed = 0.025;
  }

  public static class KitBotConstants {
    // CAN IDs for the Spark MAX motor controllers.
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for the Spark MAX motor controllers.
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for intaking and launching. Launch speeds are negative to run the motors in reverse.
    public static final double kShootLauncherSpeed = -1.5;
    public static final double kShootFeederSpeed = -1.5;
    public static final double kIntakeLauncherSpeed = 1;
    public static final double kIntakeFeederSpeed = .2;

    // Delay for starting the feeder wheel to allow the launcher wheel to spin up.
    public static final double kFeederDelay = 1;
  }

  public static class ClimberConstants {
    // CAN IDs for the Spark MAX motor controllers.
    public static final byte kLeftClimberID = 7;
    public static final byte kRightClimberID = 8;

    // Speed for climbing. Clmbing speed is negative to run the motors in reverse.
    public static final double kClimberSpeed = -1;

    // Current limit for the Spark MAX motor controllers.
    public static final int kClimberCurrentLimit = 25;
  }

  // Data for each individual swerve module.
  public static class SwerveConstants {
    // Gains for module velocity error -> voltage.
    public static final TunableNumber kDriveS = new TunableNumber("S", 0.1, "Module Drive");
    public static final TunableNumber kDriveP = new TunableNumber("P", 2, "Module Drive");

    // Stores whether the azimuth motor is inverted; use for Mk4i modules.
    public static final boolean kAzimuthInverted = true;

    public static final SwerveModuleConstants
        kSwerve_fl =
            new SwerveModuleConstants(
                "SwerveFL",
                // Azimuth gains (kS, kP, kD).
                0.1, // 0.1
                0.094, // 0.095
                0.0008, // 0.0006
                // CANcoder offset.
                -0.190917,
                // CAN IDs (driving motor, turning motor, CANCoder).
                11,
                12,
                13,
                new Translation2d(-0.212461, -0.263261)),
        kSwerve_fr =
            new SwerveModuleConstants(
                "SwerveFR",
                // Azimuth gains (kS, kP, kD).
                0.092, // 0.092
                0.062, // 0.092
                0.00065, // 0.00065
                // CANcoder offset.
                -0.679931,
                // CAN IDs (driving motor, turning motor, CANCoder).
                21,
                22,
                23,
                new Translation2d(-0.212461, 0.263261)),
        kSwerve_bl =
            new SwerveModuleConstants(
                "SwerveBL",
                // Azimuth gains (kS, kP, kD).
                0.08, // 0.08
                0.105, // 0.105
                0.0004, // 0.0004
                // CANcoder offset.
                -0.603271,
                // CAN IDs (driving motor, turning motor, CANCoder).
                31,
                32,
                33,
                new Translation2d(0.212461, -0.263261)),
        kSwerve_br =
            new SwerveModuleConstants(
                "SwerveBR",
                // Azimuth gains (kS, kP, kD).
                0.092, // 0.092
                0.09, // 0.09
                0.00065, // 0.00065
                // CANcoder offset.
                -0.598876,
                // CAN IDs (driving motor, turning motor, CANCoder).
                41,
                42,
                43,
                new Translation2d(0.212461, 0.263261));

    // Drive base radius for angular velocity calcs (use swerve module farthest from COR).
    public static final double kDriveBaseRadius =
        kSwerve_bl.location.getDistance(new Translation2d());
  }

  public static class AutoConstants {
    // TODO (dev/user): Ensure that drivetrain acceleration limits are strong.
    // Upper bound drivetrain accelerations for path following and pose targeting.
    public static final double kMaxLinearAccelMetersPerSecondSquared =
        DriveConstants.kMaxLinearVelMetersPerSecond;
    public static final double kMaxTurnAccelRadiansPerSecondSquared =
        DriveConstants.kMaxTurnVelRadiansPerSecond;

    // TODO(user): Tune pathfinding PID gains (currently at default gains).
    // Gains for drivetrain position error -> velocity.
    public static final TunableNumber kTranslateP = new TunableNumber("P", 5, "Robot Translation");
    public static final TunableNumber kTranslateD = new TunableNumber("D", 0, "Robot Translation");
  }
}
