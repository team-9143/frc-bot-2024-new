package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public enum Body implements MutableChooser.Named {
  Escape("Escape"),
  None("None");

  final String name;

  Body(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public Command getCommand(StartPose startPose) {
    switch (this) {
      case Escape:
        if (startPose == StartPose.Wing) {
          var path =
              Pathing.generateDirectPath(
                  StartPose.Wing.pose,
                  StartPose.Wing.pose.plus(new Transform2d(6, 0, new Rotation2d())));
          // Path will be flipped automatically during this call
          return Pathing.getHolonomicFollowPathCommand(path);
        }
        return Pathing.getHolonomicFollowPathCommand(Pathing.loadPath(startPose.getName()));

      default:
        return new InstantCommand();
    }
  }
}
