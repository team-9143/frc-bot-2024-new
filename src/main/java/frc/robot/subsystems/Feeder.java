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
    m_feedMotor.optimizeBusUtilization();
  }

  // TODO(shooter): Create a private constructor here to initialize the talon (netural mode and current limits)
  // TODO(shooter): Add a default command that holds a game piece in the feeder wheels while a note is within the bot (between intake and shooting)
  private Feeder() {}

  /** @return a command to feed a note from the intake to the shooter */
  public Command getFeedCommand() {
    return startEnd(
      () -> m_feedMotor.setVoltage(FeederConsts.kFeedVolts),
      this::stop
    );
  }

  @Override
  public void stop() {
    System.out.println("Stopping feeder");
    m_feedMotor.stopMotor();
  }

  @Override
  public void log() {
    // TODO(shooter): I can't think of anything to log here, you could leave this method empty
  }
}