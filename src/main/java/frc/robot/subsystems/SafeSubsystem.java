package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logger.Loggable;
import frc.robot.logger.Logger;
import java.util.ArrayList;

/** Subsystem with a stop method for safety measures. */
public abstract class SafeSubsystem extends SubsystemBase implements Loggable {
  private static final ArrayList<SafeSubsystem> s_subsystems = new ArrayList<>();

  public SafeSubsystem() {
    s_subsystems.add(this);
    Logger.registerLoggable(this);
  }

  /**
   * @return a list of all initalized subsystems
   */
  public static SafeSubsystem[] getAll() {
    return s_subsystems.toArray(SafeSubsystem[]::new);
  }

  @Override
  public String getDirectory() {
    return "/" + getName() + "/";
  }

  /** Safe state method. Should be called in emergencies and when disabled. */
  public abstract void stop();
}
