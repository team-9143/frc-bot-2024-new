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

  // TODO(shooter): Make this private for safety
  public final static TalonFX m_feedMotor = new TalonFX(FeederConsts.kFeedMotorID);

  // TODO(shooter): Create a private constructor here to initialize the talon

  // TODO(shooter): There should be a feed command and a hold command, add and document both properly
  /** @return a command to feed a note from the intake to the shooter */
  public Command getFeedCommand() {
    return startEnd(
      () -> {
        m_feedMotor.setVoltage(FeederConsts.kHoldVolts);
      },
      Shooter.getInstance()::stop // TODO(shooter): Change these all into `this::stop` (more readable and less processing)
    );
  }

  @Override
  public void stop() {
    m_feedMotor.stopMotor();
  }

  @Override
  public void log() {
    // TODO(shooter): I can't think of anything to log here, you could leave this method empty
  }
}