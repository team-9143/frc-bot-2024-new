package frc.robot.autos;

import static com.pathplanner.lib.util.GeometryUtil.flipFieldPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/** Enum of starting positions to set drivetrain odometry and meet path expectations. */
public enum StartPose {
  SubwooferAmp("Subwoofer amp side", new Pose2d(0.88, 6.57, Rotation2d.fromDegrees(60))),
  SubwooferSource("Subwoofer source side", new Pose2d(0.88, 4.53, Rotation2d.fromDegrees(-60))),
  SubwooferFront("Subwoofer front", new Pose2d(1.36, 5.55, new Rotation2d())),
  Wing("Wing", new Pose2d(1.36, 1.62, new Rotation2d()));

  final String name;

  /** Raw unflipped pose */
  final Pose2d pose;

  StartPose(String name, Pose2d pose) {
    this.name = name;
    this.pose = pose;
  }

  public String toString() {
    return name;
  }

  /** Resets drivetrain odometry to assumed starting pose */
  public Command getCommand() {
    return new InstantCommand(() -> Drivetrain.resetOdometry(getPose()));
  }

  /** Returns correctly flipped pose */
  public Pose2d getPose() {
    return Pathing.isRedAlliance() ? flipFieldPose(pose) : pose;
  }
}
