package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public enum Starter implements MutableChooser.Named {
  Shoot("Shoot"),
  WaitToShoot("Wait and shoot"),
  None("None");

  final String name;

  Starter(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public Command getCommand() {
    switch (this) {
      case Shoot:
        return getShootCommand();

      case WaitToShoot:
        return new WaitCommand(0.5).andThen(getShootCommand());

      default:
        return new InstantCommand();
    }
  }

  private static Command getShootCommand() {
    return Shooter.getInstance().getShootCommand().withTimeout(2);
  }
}
