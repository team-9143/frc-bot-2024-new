package frc.robot.autos;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/** Enum of starting positions to set drivetrain odometry and meet path expectations. */
public enum StartPose {
  Front_Side_V2(new Pose2d(1.36, 5.55, new Rotation2d())),
  Source_Side_V3(new Pose2d(0.75, 4.46, Rotation2d.fromDegrees(-60))),
  Amp_Side_V4(new Pose2d(0.75, 6.62, Rotation2d.fromDegrees(60))),
  DNU_Source_Side_Short(new Pose2d(0.75, 4.46, Rotation2d.fromDegrees(-60))),

  Wing(new Pose2d(1.36, 1.62, new Rotation2d()));

  /** Raw unflipped pose */
  private final Pose2d pose;

  StartPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Resets drivetrain odometry to assumed starting pose */
  public Command getCommand() {
    return new InstantCommand(() -> Drivetrain.resetOdometry(getDSRelativePose()));
  }

  /** Returns raw unflipped pose */
  public Pose2d getRawPose() {
    return pose;
  }

  /**
   * Returns pose as seen from driver station rotation (used for driving, matches with ds- and
   * robot-oriented movement)
   */
  public Pose2d getDSRelativePose() {
    return Pathing.isRedAlliance() ? GeometryUtil.flipFieldPose(pose) : pose;
  }
}
