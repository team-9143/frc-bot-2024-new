package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public enum Body implements MutableChooser.Named {
  MoveBack("Run away 6m"),
  EscapeSubwoofer("Escape from subwoofer"),
  None("None");

  final String name;

  Body(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public Command getAuto(StartPose startPose) {
    switch (this) {
      case MoveBack:
        return Pathing.getHolonomicTargetPoseCommand(
            Drivetrain.getPose().plus(new Transform2d(6, 0, new Rotation2d())));

      case EscapeSubwoofer:
        return Pathing.getHolonomicFollowPathCommand(Pathing.loadPath(startPose.getName()));

      default:
        return new InstantCommand();
    }
  }
}
