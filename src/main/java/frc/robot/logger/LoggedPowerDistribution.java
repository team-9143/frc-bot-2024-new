package frc.robot.logger;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;

public class LoggedPowerDistribution extends PowerDistribution implements Loggable {
  private static final String LOG_DIR = "/power/";

  // Faults to compare against to log differences
  private PowerDistributionFaults prevFaults = new PowerDistributionFaults(0);

  /** Detects the connected PDP/PDH using the default CAN ID (0 for CTRE and 1 for REV). */
  public LoggedPowerDistribution() {
    super();

    // Register for periodic logging
    Logger.registerLoggable(this);
  }

  @Override
  public String getDirectory() {
    return LOG_DIR;
  }

  @Override
  public void log() {
    PowerDistributionFaults currFaults = this.getFaults();

    Logger.recordOutput(getDirectory() + "inputVoltage", this.getVoltage());
    logIfDifferent(getDirectory() + "brownout", prevFaults.Brownout, currFaults.Brownout);
    logIfDifferent(getDirectory() + "CANWarning", prevFaults.CanWarning, currFaults.CanWarning);
  }

  private void logIfDifferent(String path, boolean prev, boolean curr) {
    if (prev != curr) {
      Logger.recordOutput(path, curr);
    }
  }
}
