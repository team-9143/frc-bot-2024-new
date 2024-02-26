package frc.robot.subsystems;

import frc.robot.logger.Logger;

// TODO(user): Remove
/** And example state-based subsystem that serves no purpose whatsoever. */
public class ExampleStateSubsystem extends StateSubsystem<ExampleStateSubsystem.States> {
  private static ExampleStateSubsystem m_instance;

  /** @return the singleton instance */
  public static synchronized ExampleStateSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ExampleStateSubsystem();
      m_instance.stop();
    }
    return m_instance;
  }

  /** Initialization (e.g. motors and default command). */
  private ExampleStateSubsystem() {}

  /** Main logic method to run methods based on current state. */
  @Override
  public void periodic() {}

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"state", getState().name());
  }

  @Override
  public void stop() {
    setState(States.STOP);
  }

  public enum States {
    FAST,
    SLOW,
    STOP
  }
}