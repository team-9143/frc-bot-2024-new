package frc.robot.subsystems;

import frc.robot.logger.Logger;
import frc.robot.util.SwerveDrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;

import frc.robot.devices.OI;
import frc.robot.Constants.Config;
import frc.robot.Constants.DeviceConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.SwerveConsts;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Controls the robot drivetrain. */
public class Drivetrain extends SafeSubsystem {
  private static Drivetrain m_instance;

  /** @return the singleton instance */
  public static synchronized Drivetrain getInstance() {
    if (m_instance == null) {
      m_instance = new Drivetrain();
      m_instance.stop();
    }
    return m_instance;
  }

  // Pigeon2 setup
  private static final Pigeon2 m_pigeon2 = new Pigeon2(DeviceConsts.kPigeonID);
  static {
    m_pigeon2.getConfigurator().apply(Config.kPigeonMountPose);
    m_pigeon2.setYaw(0);
    StatusSignal.setUpdateFrequencyForAll(50, m_pigeon2.getYaw(), m_pigeon2.getQuatW(), m_pigeon2.getQuatX(), m_pigeon2.getQuatY(), m_pigeon2.getQuatZ());
    m_pigeon2.optimizeBusUtilization();
  }
  private static Rotation3d gyroOffset; // To adjust 3d rotation (quaternion from gyro) to match with 2d odometry after a heading reset

  public static final SwerveModuleState[] xStanceStates = new SwerveModuleState[] {
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
  };

  private static final SwerveDrive m_swerve = new SwerveDrive(
    m_pigeon2::getRotation2d,
    SwerveConsts.kSwerve_fl,
    SwerveConsts.kSwerve_fr,
    SwerveConsts.kSwerve_bl,
    SwerveConsts.kSwerve_br
  );

  private Drivetrain() {
    // Default drive command
    setDefaultCommand(run(() -> {
      double forward = -OI.DRIVER_CONTROLLER.getLeftY();
      double left = -OI.DRIVER_CONTROLLER.getLeftX();
      double ccw = -OI.DRIVER_CONTROLLER.getRightX(); // Right joystick horizontal for rotation

      // Field relative control, exponentially scaling inputs to increase sensitivity
      this.driveFieldRelativeVelocity(
        Math.copySign(forward*forward, forward) * DriveConsts.kMaxLinearVelMetersPerSecond * DriveConsts.kTeleopSpeedMult,
        Math.copySign(left*left, left) * DriveConsts.kMaxLinearVelMetersPerSecond * DriveConsts.kTeleopSpeedMult,
        Math.copySign(ccw*ccw*ccw, ccw) * DriveConsts.kMaxTurnVelRadiansPerSecond * DriveConsts.kTeleopTurnMult // Extra sensitivity for fine rotation control
      );
    }));
  }


  @Override
  public void periodic() {
    // Update swerve speeds and odometry
    m_swerve.update();
  }

  /**
   * Drive with field relative velocities. Must be continuously called. This method is intended for general teleop drive use.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: ccw radians/s)
   */
  public void driveFieldRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocityRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, ccw, getPose().getRotation()));
  }

  /**
   * Drive with field relative velocities. Must be continuously called.
   *
   * @param speeds {@link ChassisSpeeds} object in meters/s
   */
  public void driveFieldRelativeVelocity(ChassisSpeeds speeds) {
    m_swerve.setDesiredVelocityRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
  }

  /**
   * Drive with robot relative velocities. Must be continuously called.
   *
   * @param speeds {@link ChassisSpeeds} object in meters/s
   */
  public void driveRobotRelativeVelocity(ChassisSpeeds speeds) {
    m_swerve.setDesiredVelocityRobotRelative(speeds);
  }

  /** Set the drivetrain to x-stance for traction. Must be continuously called. */
  public void toXStance() {
    m_swerve.setDesiredStates(
      xStanceStates[0],
      xStanceStates[1],
      xStanceStates[2],
      xStanceStates[3]
    );
  }

  /**
   * Reset the odometry to a given position.
   *
   * @param positionMetersCCW robot position (UNIT: meters, ccw native angle)
   */
  public void resetOdometry(Pose2d positionMetersCCW) {
    var gyroAngle = m_pigeon2.getRotation2d();

    gyroOffset = new Rotation3d(0, 0, getPose().getRotation().minus(gyroAngle).getRadians());
    m_swerve.resetOdometry(positionMetersCCW, gyroAngle);
  }

  /** @return the robot's estimated location */
  public Pose2d getPose() {return m_swerve.getPose();}

  /** @return the gyro's orientation */
  public Rotation3d getOrientation() {return m_pigeon2.getRotation3d().plus(gyroOffset);}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return m_swerve.getDesiredSpeeds();}

  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getMeasuredSpeeds() {return m_swerve.getMeasuredSpeeds();}

  /** @return individual desired module states */
  public SwerveModuleState[] getDesiredStates() {return m_swerve.getDesiredStates();}

  /** @return individual measured module states */
  public SwerveModuleState[] getMeasuredStates() {return m_swerve.getMeasuredStates();}

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"odometry", getPose());

    Logger.recordOutput(getDirectory()+"measuredStates", getMeasuredStates());
    Logger.recordOutput(getDirectory()+"desiredStates", getDesiredStates());

    Logger.recordOutput(getDirectory()+"measuredSpeeds", getMeasuredSpeeds());
    Logger.recordOutput(getDirectory()+"desiredSpeeds", getDesiredSpeeds());

    Logger.recordOutput(getDirectory()+"3dPosition", new Pose3d(getPose().getX(), getPose().getY(), 0, getOrientation())); // Height always set to 0

    // Uncoment to log azimuth errors
    // Logger.recordOutput(getDirectory()+"AngleErrorFL", m_swerve.modules[0].getAngleError());
    // Logger.recordOutput(getDirectory()+"AngleErrorFR", m_swerve.modules[1].getAngleError());
    // Logger.recordOutput(getDirectory()+"AngleErrorBL", m_swerve.modules[2].getAngleError());
    // Logger.recordOutput(getDirectory()+"AngleErrorBR", m_swerve.modules[3].getAngleError());

    // Uncomment to log velocity errors
    // Logger.recordOutput(getDirectory()+"VelErrorFL", getDesiredStates()[0].speedMetersPerSecond - m_swerve.modules[0].getVelocity());
    // Logger.recordOutput(getDirectory()+"VelErrorFR", getDesiredStates()[1].speedMetersPerSecond - m_swerve.modules[1].getVelocity());
    // Logger.recordOutput(getDirectory()+"VelErrorBL", getDesiredStates()[2].speedMetersPerSecond - m_swerve.modules[2].getVelocity());
    // Logger.recordOutput(getDirectory()+"VelErrorBR", getDesiredStates()[3].speedMetersPerSecond - m_swerve.modules[3].getVelocity());
  }

  @Override
  public void stop() {
    m_swerve.stopMotor();
  }
}