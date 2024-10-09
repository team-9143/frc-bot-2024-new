package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.KitBot;

/** Enum of starting actions to deal with the preloaded game piece. */
public enum Starter {
  Shoot,
  Wait_and_shoot,
  None;

  public Command getCommand() {
    switch (this) {
      case Shoot:
        return getShootCommand().withTimeout(0.25);

      case Wait_and_shoot:
        return new WaitCommand(0.5).andThen(getShootCommand().withTimeout(0.25));

      default:
        return new InstantCommand();
    }
  }

  // Returns shoot command with both launcher and feeder wheels
  public static Command getShootCommand() {
    return KitBot.getInstance().getShootCommand();
  }
}
