package frc.robot.subsystems;

import frc.robot.Constants.ShooterConsts;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/** Controls indexer and shooter wheels. */
public class Shooter extends SafeSubsystem {
  private static Shooter m_instance;

  /** @return the singleton instance */
  public static synchronized Shooter getInstance() {
    if (m_instance == null) {
      m_instance = new Shooter();
    }
    return m_instance;
  }

  public final static CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);

  public final static CANSparkMax m_ShooterTop = new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless);


  /** @return a command to intake a game piece using shooter wheels */
  public Command getSourceIntakeCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(-ShooterConsts.kSourceIntakeVolts);
      },
      Shooter.getInstance()::stop
    );
  }

  /** @return a command to shoot a game piece using shooter wheels */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_ShooterTop.setVoltage(-ShooterConsts.kShootVolts);
      },
      Shooter.getInstance()::stop
    );
  }

  /** @return a command to spit a game piece at partial speed for amp */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(ShooterConsts.kSpitVolts);

      },
      Shooter.getInstance()::stop
    );
  }

  @Override
  public void stop() {
    m_ShooterBottom.stopMotor();
    m_ShooterTop.stopMotor();
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    
  }
    
}