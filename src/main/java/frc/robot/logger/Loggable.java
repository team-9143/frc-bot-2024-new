package frc.robot.logger;

/** Interface to create loggable devices that can be updated by a call from the Logger class. */
public interface Loggable {
  /**
   * @return directory where device data will be logged (with trailing slash)
   */
  public String getDirectory();

  /** Log all relevant data with the Logger. */
  public void log();
}
