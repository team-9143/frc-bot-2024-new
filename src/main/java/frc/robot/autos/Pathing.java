package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.logger.Logger;
import frc.robot.subsystems.Drivetrain;
import java.util.Collections;
import java.util.List;

/**
 * Utility class for loading, generating, following, and logging paths through PathPlanner. PID
 * controllers are initialized at runtime so that TunableNumbers can take effect.
 */
public class Pathing {
  private static final String PATH_LOG_DIR = "/pathplanner/";

  /** Default constraints for accurately generating and following paths */
  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(
          DriveConsts.kMaxLinearVelMetersPerSecond,
          AutoConsts.kMaxLinearAccelMetersPerSecondSquared,
          DriveConsts.kMaxTurnVelRadiansPerSecond,
          AutoConsts.kMaxTurnAccelRadiansPerSecondSquared);

  // Set up logging for basic path following
  static {
    // Log trajectory on command initialization
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput(PATH_LOG_DIR + "activePath", poses.toArray(Pose2d[]::new)));

    // Log target pose during command run
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput(PATH_LOG_DIR + "targetPose", pose));

    // No need to log current robot pose, drivetrain will do that
    PathPlannerLogging.setLogCurrentPoseCallback(null);
  }

  // Configure AutoBuilder
  static {
    AutoBuilder.configureHolonomic(
        Drivetrain::getPose, // Pose supplier
        Drivetrain::resetOdometry, // Reset pose consumer
        Drivetrain::getDesiredSpeeds, // Current measured speeds
        Drivetrain::driveFieldRelativeVelocity, // Drives field relative from ChassisSpeeds
        getHolonomicConfig(new ReplanningConfig(false, false)), // Config
        Pathing::isRedAlliance, // Flip if alliance is red
        Drivetrain.getInstance() // Subsystem
        );
  }

  /******
   *
   * Path loading and generation
   *
   ******/

  /**
   * Load a pathplanner path from RIO local storage
   *
   * @param name name of the path file under [deploy/pathplanner/paths/], omitting ".path"
   */
  public static PathPlannerPath loadPath(String name) {
    return PathPlannerPath.fromPathFile(name);
  }

  /**
   * Load a choreo trajectory from RIO local storage
   *
   * @param name name of the path file under [deploy/choreo/], omitting ".traj"
   */
  public static PathPlannerPath loadChoreoTraj(String name) {
    return PathPlannerPath.fromChoreoTrajectory(name);
  }

  /**
   * Create a pathplanner path to move directly from one position to another
   *
   * @param startPoseMetersCCW starting position for the command
   * @param endPoseMetersCCW ending position for the command
   */
  public static PathPlannerPath generateDirectPath(
      Pose2d startPoseMetersCCW, Pose2d endPoseMetersCCW) {
    return new PathPlannerPath(
        List.of(
            startPoseMetersCCW.getTranslation(),
            startPoseMetersCCW.getTranslation(),
            endPoseMetersCCW.getTranslation(),
            endPoseMetersCCW.getTranslation()),
        Collections.emptyList(),
        Collections.emptyList(),
        Collections.emptyList(),
        DEFAULT_CONSTRAINTS,
        new GoalEndState(0, endPoseMetersCCW.getRotation()),
        false);
  }

  /******
   *
   * Path following
   *
   ******/

  /**
   * Create a command to follow a pathplanner path. Does not replan the path under any
   * circumstances.
   *
   * @param path the pathplanner path to follow
   */
  public static Command getHolonomicFollowPathCommand(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  /**
   * Create a command to follow a pathplanner path. Allows for customized replanning settings.
   *
   * @param path the pathplanner path to follow
   * @param replanningConfig replanning configuration to use
   */
  public static Command getHolonomicFollowPathCommand(
      PathPlannerPath path, ReplanningConfig replanningConfig) {
    return new FollowPathHolonomic(
        path, // Path to follow
        Drivetrain::getPose, // Pose supplier
        Drivetrain::getMeasuredSpeeds, // Chassis speeds supplier
        Drivetrain::driveRobotRelativeVelocity, // Robot-relative velocities consumer
        getHolonomicConfig(replanningConfig), // Follower configuration
        Pathing::isRedAlliance, // Flip the path if alliance is red
        Drivetrain.getInstance() // Subsystem requirements
        );
  }

  /**
   * Create a command to follow a pathplanner auto. Only replans the path if the initial error is
   * too large.
   *
   * @param name name of the path file under [deploy/pathplanner/autos/], omitting ".auto"
   */
  public static Command getHolonomicFullAutoCommand(String name) {
    return AutoBuilder.buildAuto(name);
  }

  /**
   * Create a command that follows a path directly to the target pose and stops. Command should be
   * scheduled before any other movement happens.
   *
   * @param targetPoseMetersCCW target position for the command
   */
  public static Command getHolonomicTargetPoseCommand(Pose2d targetPoseMetersCCW) {
    var path = generateDirectPath(Drivetrain.getPose(), targetPoseMetersCCW);
    path.preventFlipping = true;
    return getHolonomicFollowPathCommand(path);
  }

  /**
   * Create a command that pathfinds to a path and then follows that path.
   *
   * <p>Pathfinding may take excessive processing, prefer running a path follower command in
   * sequence after {@link Pathing#getHolonomicTargetPoseCommand(Pose2d)}.
   *
   * @param path the pathplanner path to target and follow
   */
  public static Command getHolonomicTargetPathCommand(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(path, DEFAULT_CONSTRAINTS);
  }

  /******
   *
   * Utility
   *
   ******/

  /**
   * Utility method to get a path follower config for the swerve drivetrain
   *
   * @param replanningConfig replanning configuration to use
   * @return the configuration
   */
  private static HolonomicPathFollowerConfig getHolonomicConfig(ReplanningConfig replanningConfig) {
    return new HolonomicPathFollowerConfig(
        new PIDConstants(
            AutoConsts.kTranslateP.getAsDouble(),
            AutoConsts.kTranslateI.getAsDouble(),
            AutoConsts.kTranslateD
                .getAsDouble()), // Translation controller for position error -> velocity
        new PIDConstants(
            AutoConsts.kRotateP.getAsDouble(),
            AutoConsts.kRotateI.getAsDouble(),
            AutoConsts.kRotateD
                .getAsDouble()), // Rotation controller for angle error -> angular velocity
        DriveConsts.kMaxLinearVelMetersPerSecond, // Maximum module speed
        frc.robot.Constants.SwerveConsts.kSwerve_fl.location.getDistance(
            new Translation2d()), // Radius of drive base
        replanningConfig // When to replan the path
        );
  }

  /**
   * Utility method to check driver station alliance
   *
   * @return {@code true} if alliance is red and not null
   */
  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }
}
