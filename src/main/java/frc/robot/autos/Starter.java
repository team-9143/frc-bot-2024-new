package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/** Enum of starting actions to deal with the preloaded game piece. */
public enum Starter {
  Shoot,
  Wait_and_shoot,
  None;

  public Command getCommand() {
    switch (this) {
      case Shoot:
        return getFullShootCommand().withTimeout(2);

      case Wait_and_shoot:
        return new WaitCommand(0.5).andThen(getFullShootCommand().withTimeout(2));

      default:
        return new InstantCommand();
    }
  }

  /** Returns full shoot command with both shooter and feeder wheels, public for operator use */
  public static Command getFullShootCommand() {
    return Shooter.getInstance()
        .getShootCommand()
        .alongWith(new WaitCommand(0.5).andThen(Feeder.getInstance().getFeedUpCommand()));
  }
}
