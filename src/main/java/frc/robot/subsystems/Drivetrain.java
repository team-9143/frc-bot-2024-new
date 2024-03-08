package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Config;
import frc.robot.Constants.DeviceConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.devices.OI;
import frc.robot.logger.Logger;
import frc.robot.util.SwerveDrive;

/** Controls the robot drivetrain. */
public class Drivetrain extends SafeSubsystem {
  // Pigeon2 setup
  private static final Pigeon2 m_pigeon2 = new Pigeon2(DeviceConsts.kPigeonID);

  static {
    m_pigeon2.getConfigurator().apply(Config.kPigeonMountPose);
    m_pigeon2.setYaw(0);
    StatusSignal.setUpdateFrequencyForAll(
        1000d / DriveConsts.kPeriodMs,
        m_pigeon2.getYaw(),
        m_pigeon2.getQuatW(),
        m_pigeon2.getQuatX(),
        m_pigeon2.getQuatY(),
        m_pigeon2.getQuatZ());
    m_pigeon2.optimizeBusUtilization();
  }

  // TODO: Fix gyro offset problem with 3d rotation
  /**
   * To adjust 3d rotation (quaternion from gyro) to match with 2d odometry after a heading reset
   */
  private static Rotation3d gyroOffset = new Rotation3d();

  public static final SwerveModuleState[] xStanceStates =
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      };

  private static final SwerveDrive m_swerve =
      new SwerveDrive(
          m_pigeon2::getRotation2d,
          SwerveConsts.kSwerve_fl,
          SwerveConsts.kSwerve_fr,
          SwerveConsts.kSwerve_bl,
          SwerveConsts.kSwerve_br);

  private static final Drivetrain m_instance = new Drivetrain();

  /** Returns the singleton instance */
  public static Drivetrain getInstance() {
    return m_instance;
  }

  private Drivetrain() {
    // Default drive command
    setDefaultCommand(
        run(
            () -> {
              // Left joystick for translation, right joystick for rotation
              double forward = -OI.DRIVER_CONTROLLER.getLeftY();
              double left = -OI.DRIVER_CONTROLLER.getLeftX();
              double ccw = -OI.DRIVER_CONTROLLER.getRightX();

              // Field relative control, exponentially scaling inputs to increase sensitivity
              driveFieldRelativeVelocity(
                  Math.copySign(forward * forward, forward)
                      * DriveConsts.kMaxLinearVelMetersPerSecond
                      * DriveConsts.kTeleopSpeedMult,
                  Math.copySign(left * left, left)
                      * DriveConsts.kMaxLinearVelMetersPerSecond
                      * DriveConsts.kTeleopSpeedMult,
                  // Extra sensitivity for finer rotation control
                  Math.copySign(ccw * ccw * ccw, ccw)
                      * DriveConsts.kMaxTurnVelRadiansPerSecond
                      * DriveConsts.kTeleopTurnMult);
            }));
  }

  /**
   * Updates the swerve module states and drivetrain odometry. Should be called as often as
   * possible.
   */
  public static void update() {
    m_swerve.updateSpeeds();
    m_swerve.updateOdometry();
  }

  /**
   * Drive with field relative velocities. Must be continuously called. This method is intended for
   * general teleop drive use.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: ccw radians/s)
   */
  public static void driveFieldRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocityRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, ccw, getPose().getRotation()));
  }

  /**
   * Drive with field relative velocities. Must be continuously called.
   *
   * @param speeds {@link ChassisSpeeds} object in meters/s
   */
  public static void driveFieldRelativeVelocity(ChassisSpeeds speeds) {
    m_swerve.setDesiredVelocityRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
  }

  /**
   * Drive with robot relative velocities. Must be continuously called.
   *
   * @param speeds {@link ChassisSpeeds} object in meters/s
   */
  public static void driveRobotRelativeVelocity(ChassisSpeeds speeds) {
    m_swerve.setDesiredVelocityRobotRelative(speeds);
  }

  /** Set the drivetrain to x-stance for traction. Must be continuously called. */
  public static void toXStance() {
    m_swerve.setDesiredStates(
        xStanceStates[0], xStanceStates[1], xStanceStates[2], xStanceStates[3]);
  }

  /**
   * Reset the odometry to a given position.
   *
   * @param positionMetersCCW robot position (UNIT: meters, ccw native angle)
   */
  public static void resetOdometry(Pose2d positionMetersCCW) {
    var gyroAngle = m_pigeon2.getRotation2d();

    gyroOffset = new Rotation3d(0, 0, getPose().getRotation().minus(gyroAngle).getRadians());
    m_swerve.resetOdometry(positionMetersCCW, gyroAngle);
  }

  /** Returns the robot's estimated location */
  public static Pose2d getPose() {
    return m_swerve.getPose();
  }

  /** Returns the gyro's orientation */
  public static Rotation3d getOrientation() {
    return m_pigeon2.getRotation3d().plus(gyroOffset);
  }

  /** Returns the drivetrain's desired velocities */
  public static ChassisSpeeds getDesiredSpeeds() {
    return m_swerve.getDesiredSpeeds();
  }

  /** Returns the drivetrain's actual velocities, as measured by encoders */
  public static ChassisSpeeds getMeasuredSpeeds() {
    return m_swerve.getMeasuredSpeeds();
  }

  /** Returns individual desired module states */
  public static SwerveModuleState[] getDesiredStates() {
    return m_swerve.getDesiredStates();
  }

  /** Returns individual measured module states */
  public static SwerveModuleState[] getMeasuredStates() {
    return m_swerve.getMeasuredStates();
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory() + "odometry", getPose());

    Logger.recordOutput(getDirectory() + "measuredStates", getMeasuredStates());
    Logger.recordOutput(getDirectory() + "desiredStates", getDesiredStates());

    Logger.recordOutput(getDirectory() + "measuredSpeeds", getMeasuredSpeeds());
    Logger.recordOutput(getDirectory() + "desiredSpeeds", getDesiredSpeeds());

    // 3d pose with height always set to 0
    Logger.recordOutput(
        getDirectory() + "3dPosition",
        new Pose3d(getPose().getX(), getPose().getY(), 0, getOrientation()));

    // Uncoment to log azimuth errors
    // Logger.recordOutput(getDirectory()+"AngleErrorFL", m_swerve.modules[0].getAngleError());
    // Logger.recordOutput(getDirectory()+"AngleErrorFR", m_swerve.modules[1].getAngleError());
    // Logger.recordOutput(getDirectory()+"AngleErrorBL", m_swerve.modules[2].getAngleError());
    // Logger.recordOutput(getDirectory()+"AngleErrorBR", m_swerve.modules[3].getAngleError());

    // Uncomment to log velocity errors
    // Logger.recordOutput(
    //     getDirectory() + "VelErrorFL",
    //     getDesiredStates()[0].speedMetersPerSecond - m_swerve.modules[0].getVelocity());
    // Logger.recordOutput(
    //     getDirectory() + "VelErrorFR",
    //     getDesiredStates()[1].speedMetersPerSecond - m_swerve.modules[1].getVelocity());
    // Logger.recordOutput(
    //     getDirectory() + "VelErrorBL",
    //     getDesiredStates()[2].speedMetersPerSecond - m_swerve.modules[2].getVelocity());
    // Logger.recordOutput(
    //     getDirectory() + "VelErrorBR",
    //     getDesiredStates()[3].speedMetersPerSecond - m_swerve.modules[3].getVelocity());
  }

  @Override
  public void stop() {
    m_swerve.stopMotor();
  }
}
