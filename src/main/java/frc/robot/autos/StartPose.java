package frc.robot.autos;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/** Enum of starting positions to set drivetrain odometry and meet path expectations. */
public enum StartPose {
  Subwoofer_amp_side(new Pose2d(0.88, 6.57, Rotation2d.fromDegrees(60))),
  Subwoofer_source_side(new Pose2d(0.88, 4.53, Rotation2d.fromDegrees(-60))),
  Subwoofer_front(new Pose2d(1.36, 5.55, new Rotation2d())),
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
    return Pathing.isRedAlliance()
        ? new Pose2d(
            GeometryUtil.flipFieldPosition(pose.getTranslation()), pose.getRotation().unaryMinus())
        : pose;
  }
}
