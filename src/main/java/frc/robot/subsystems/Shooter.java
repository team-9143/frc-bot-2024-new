package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.ShooterConsts;

/** Controls shooter wheels. */
public class Shooter extends SafeSubsystem {
  private static Shooter m_instance;

  /** @return the singleton instance */
  public static synchronized Shooter getInstance() {
    if (m_instance == null) {
      m_instance = new Shooter();
    }
    return m_instance;
  }

  private final static CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);
  static {
    new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless).follow(m_ShooterBottom, true);

    m_ShooterBottom.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit);
  }

  // TODO(shooter): Create a private constructor here to initialize the encoders
  private Shooter() {}

  /** @return a command to intake a game piece using shooter wheels */
  public Command getSourceIntakeCommand() {
    return startEnd(
      () -> m_ShooterBottom.setVoltage(ShooterConsts.kSourceIntakeVolts),
      this::stop
    );
  }

  /** @return a command to shoot a game piece using shooter wheels */
  public Command getShootCommand() {
    return startEnd(
      () -> m_ShooterBottom.setVoltage(ShooterConsts.kShootVolts),
      this::stop
    );
  }

  /** @return a command to spit a game piece at partial speed for amp scoring */
  public Command getSpitCommand() {
    return startEnd(
      () -> m_ShooterBottom.setVoltage(ShooterConsts.kSpitVolts),
      this::stop
    );
  }

  @Override
  public void stop() {
    System.out.println("Stopping shooter");
    m_ShooterBottom.stopMotor();
  }

  @Override
  public void log() {
    // TODO(shooter) implement wheel speed logging (use integrated NEO encoders)
  }
}