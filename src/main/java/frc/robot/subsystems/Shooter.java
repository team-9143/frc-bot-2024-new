package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
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

  // TODO(shooter): Make these private for safety
  public final static CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);
  public final static CANSparkMax m_ShooterTop = new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless);

  // TODO(shooter): Create a private constructor here to set current limits for the sparks and initialize the encoders

  /** @return a command to intake a game piece using shooter wheels */
  public Command getSourceIntakeCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(ShooterConsts.kSourceIntakeVolts);
      },
      Shooter.getInstance()::stop // TODO(shooter): Change these all into `this::stop` (more readable and less processing)
    );
  }

  /** @return a command to shoot a game piece using shooter wheels */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_ShooterTop.setVoltage(ShooterConsts.kShootVolts);
      },
      Shooter.getInstance()::stop // TODO(shooter): Change these all into `this::stop` (more readable and less processing)
    );
  }

  /** @return a command to spit a game piece at partial speed for amp scoring */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(ShooterConsts.kSpitVolts);
      },
      Shooter.getInstance()::stop // TODO(shooter): Change these all into `this::stop` (more readable and less processing)
    );
  }

  @Override
  public void stop() {
    m_ShooterBottom.stopMotor();
    m_ShooterTop.stopMotor();
  }

  @Override
  public void log() {
    // TODO(shooter) implement wheel speed logging (use integrated NEO encoders)
  }
}