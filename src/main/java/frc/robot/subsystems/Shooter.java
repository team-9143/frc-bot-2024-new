package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.ShooterConsts;
import frc.robot.devices.OI;
import frc.robot.util.SparkUtils;

/** Controls shooter wheels. */
public class Shooter extends SafeSubsystem {
  private static Shooter m_instance;

  /** Returns the singleton instance */
  public static synchronized Shooter getInstance() {
    if (m_instance == null) {
      m_instance = new Shooter();
    }
    return m_instance;
  }

  private static final CANSparkMax m_motor =
      new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless);

  static {
    // Follower setup
    @SuppressWarnings("resource")
    var follower = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);
    SparkUtils.configure(
        follower,
        () -> follower.setIdleMode(IdleMode.kCoast),
        () -> follower.follow(m_motor),
        () -> SparkUtils.setPeriodicFrames(follower, 10, 0, 0, 0, 0, 0, 0));

    // Main motor setup
    SparkUtils.configure(
        m_motor,
        () -> m_motor.setIdleMode(IdleMode.kCoast),
        () -> m_motor.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(m_motor, 10, 0, 0, 0, 0, 0, 0));
  }

  // TODO(shooter): Extra - Create a private constructor here to initialize the encoders
  private Shooter() {
    // Default command will run shooter relative to trigger velocity
    setDefaultCommand(run(() -> m_motor.setVoltage(OI.OPERATOR_CONTROLLER.getTriggers() * 12)));
  }

  /** Returns a command to intake a game piece using shooter wheels */
  public Command getSourceIntakeCommand() {
    return startEnd(() -> m_motor.setVoltage(ShooterConsts.kSourceIntakeVolts), this::stop);
  }

  /** Returns a command to shoot a game piece using shooter wheels */
  public Command getShootCommand() {
    return startEnd(() -> m_motor.setVoltage(ShooterConsts.kShootVolts), this::stop);
  }

  /** Returns a command to spit a game piece at partial speed for amp scoring */
  public Command getSpitCommand() {
    return startEnd(() -> m_motor.setVoltage(ShooterConsts.kSpitVolts), this::stop);
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void log() {
    // TODO(shooter): Extra - implement wheel speed logging (use integrated NEO encoders)
  }
}
