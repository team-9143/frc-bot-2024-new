package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Contains auto types, choosers, and compiler. */
public class AutoSelector {
  /** Returns a full auto routine */
  public static Command getAuto() {
    return new InstantCommand();
  }

  static {
    NamedCommands.registerCommand("Empty", new InstantCommand());
  }
}
