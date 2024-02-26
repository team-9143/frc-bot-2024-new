package frc.robot.subsystems;

// TODO(user): Remove
/** An example subsystem that serves no purpose whatsoever. */
public class ExampleSubsystem extends SafeSubsystem {
  private static ExampleSubsystem m_instance;

  /** @return the singleton instance */
  public static synchronized ExampleSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ExampleSubsystem();
      m_instance.stop();
    }
    return m_instance;
  }

  /** Initialization (e.g. motors and default command). */
  private ExampleSubsystem() {}

  /** Update and calculate methods that should be run every robot loop. */
  @Override
  public void periodic() {}

  @Override
  public void log() {}

  @Override
  public void stop() {}
}