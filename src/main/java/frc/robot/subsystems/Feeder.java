package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConsts;

import com.ctre.phoenix6.hardware.TalonFX;

/** Controls feeder wheels */
public class Feeder extends SafeSubsystem {
    private static Feeder m_instance;

  /** @return the singleton instance */
  public static synchronized Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  private final static TalonFX m_feedMotor = new TalonFX(FeederConsts.kFeedMotorID);
  static {
    // TODO(shooter): Extra - use the talon configurator to set current limits and neutral mode here.
    m_feedMotor.optimizeBusUtilization();
  }

  private Feeder() {}

  /** @return a command to feed a note from the intake to the shooter */
  public Command getFeedCommand() {
    return startEnd(
      () -> m_feedMotor.setVoltage(FeederConsts.kFeedVolts),
      this::stop
    );
  }

  /** @return a command to deeply intake a note from the source */
  public Command getDeepIntakeCommand() {
    return startEnd(
      () -> m_feedMotor.setVoltage(FeederConsts.kDeepIntakeVolts),
      this::stop
    );
  }

  @Override
  public void stop() {
    m_feedMotor.stopMotor();
  }

  @Override
  public void log() {}
}