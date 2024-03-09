package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public enum StartPose implements MutableChooser.Named {
  Outer("Subwoofer outer"),
  Inner("Subwoofer inner"),
  Front("Subwoofer front"),
  Wing("Wing");

  final String name;

  StartPose(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public Command getAuto() {
    switch (this) {
      case Outer:
        return new InstantCommand(
            () -> Drivetrain.resetOdometry(new Pose2d(0.88, 6.57, Rotation2d.fromDegrees(60))));
      case Inner:
        return new InstantCommand(
            () -> Drivetrain.resetOdometry(new Pose2d(0.88, 4.53, Rotation2d.fromDegrees(-60))));
      case Front:
        return new InstantCommand(
            () -> Drivetrain.resetOdometry(new Pose2d(1.36, 5.55, new Rotation2d())));
      default:
        // Arbitrary point within the starting zone very close to white tape line and source
        return new InstantCommand(
            () -> Drivetrain.resetOdometry(new Pose2d(1.36, 1.62, new Rotation2d())));
    }
  }
}
