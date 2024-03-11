package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/** Enum of starting actions to deal with the preloaded game piece. */
public enum Starter {
  Shoot("Shoot"),
  WaitToShoot("Wait and shoot"),
  None("None");

  final String name;

  Starter(String name) {
    this.name = name;
  }

  public String toString() {
    return name;
  }

  public Command getCommand() {
    switch (this) {
      case Shoot:
        return getFullShootCommand().withTimeout(2);

      case WaitToShoot:
        return new WaitCommand(0.5).andThen(getFullShootCommand().withTimeout(2));

      default:
        return new InstantCommand();
    }
  }

  public static Command getFullShootCommand() {
    return Shooter.getInstance()
        .getShootCommand()
        .alongWith(new WaitCommand(0.5).andThen(Feeder.getInstance().getFeedUpCommand()));
  }
}
