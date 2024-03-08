package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.ShooterConsts;
import frc.robot.devices.OI;
import frc.robot.logger.Logger;
import frc.robot.util.SparkUtils;

/** Controls shooter wheels. */
public class Shooter extends SafeSubsystem {
  private static final CANSparkMax m_motor =
      new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless);

  private static final RelativeEncoder encoder_top = m_motor.getEncoder();
  private static final RelativeEncoder encoder_bottom;

  static {
    // Follower setup
    var follower = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);
    encoder_bottom = follower.getEncoder();
    SparkUtils.configure(
        follower,
        () -> follower.setIdleMode(IdleMode.kCoast),
        () -> follower.follow(m_motor),
        () -> encoder_bottom.setMeasurementPeriod(20),
        () -> encoder_bottom.setVelocityConversionFactor(PhysConsts.kShooterMechToSens / 60),
        () -> SparkUtils.setPeriodicFrames(follower, 10, 20, 0, 0, 0, 0, 0));

    // Main motor setup
    SparkUtils.configure(
        m_motor,
        () -> m_motor.setIdleMode(IdleMode.kCoast),
        () -> m_motor.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit),
        () -> encoder_top.setMeasurementPeriod(20),
        () -> encoder_top.setVelocityConversionFactor(PhysConsts.kShooterMechToSens / 60),
        () -> SparkUtils.setPeriodicFrames(m_motor, 10, 20, 0, 0, 0, 0, 0));
  }

  private static final Shooter m_instance = new Shooter();

  /** Returns the singleton instance */
  public static Shooter getInstance() {
    return m_instance;
  }

  private Shooter() {
    // Default command will run shooter relative to trigger velocity
    setDefaultCommand(run(() -> m_motor.setVoltage(-OI.OPERATOR_CONTROLLER.getTriggers() * 12)));
  }

  /** Returns a command to intake a game piece using shooter wheels */
  public Command getSourceIntakeCommand() {
    return startEnd(() -> m_motor.setVoltage(ShooterConsts.kSourceIntakeVolts), this::stop);
  }

  /** Returns a command to shoot a game piece using shooter wheels */
  public Command getShootCommand() {
    return startEnd(() -> m_motor.setVoltage(ShooterConsts.kShootVolts), this::stop);
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory() + "topRPS", encoder_top.getVelocity());
    Logger.recordOutput(getDirectory() + "bottomRPS", encoder_bottom.getVelocity());
  }
}
