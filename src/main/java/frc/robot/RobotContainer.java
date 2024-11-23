package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.devices.Controller.btn;
import frc.robot.devices.OI;
import frc.robot.logger.LoggedPowerDistribution;
import frc.robot.logger.Logger;
import frc.robot.subsystems.Amper;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.KitBot;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SafeSubsystem;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

// TODO(!!!IMPORTANT!!!): Rebase & merge from the main branch. You will see a 20% drop in CAN
// utilization.

// TODO(!!!IMPORTANT!!!): Run spotless (spotlessApply under VSCode command "Gradle Build") before
// commit.

// Robot structure declaration. Initializes trigger mappings, OI devices, and main stop mechanism.

public class RobotContainer {
  private static boolean m_initialized = false;
  private static LoggedPowerDistribution powerDist;
  public final Amper amper = new Amper(); // Get instance of Amper.
  private static final LimelightSubsystem limelightSubsystem =
      new LimelightSubsystem(); // Make static

  // Initialize robot container.
  public static void init() {
    if (m_initialized == true) {
      return;
    }
    m_initialized = true;

    configureOI();
    logMetadata();
    configureBindings();
  }

  // Initialize driver station specific data.
  public static void initDS() {
    Logger.recordMetadata("Station", DriverStation.getRawAllianceStation().toString());

    Logger.recordMetadata(
        "Match",
        DriverStation.getEventName()
            + " "
            + DriverStation.getMatchType().toString()
            + " "
            + DriverStation.getMatchNumber());

    Logger.initFilename();
  }

  // Send metadata to logger.
  private static void logMetadata() {
    Logger.recordMetadata(
        "Time",
        LocalDateTime.now(ZoneId.of("UTC-8"))
            .format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")));

    Logger.recordMetadata("NT Streaming", String.valueOf(Constants.Config.NTStream));

    Logger.recordMetadata(
        "RoborioSerialNum", RobotBase.isReal() ? System.getenv("serialnum") : "Simulation");

    Logger.recordMetadata("PowerDistributionType", powerDist.getType().name());
  }

  // Initialize OI devices.
  private static void configureOI() {
    powerDist = new LoggedPowerDistribution();
    // Stop those ridiculously persistent messages
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  // Create button bindings.
  public static void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(
            () -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
        .whileTrue(
            new RunCommand(RobotContainer::stop, SafeSubsystem.getAll())
                // Interrupt incoming commands to ensure stop command takes precedence
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    configureDriver();
    configureOperator();

    // Limelight-specific bindings
    // Toggle between driver camera and vision processing
    new Trigger(() -> OI.OPERATOR_CONTROLLER.getButton(btn.Back))
        .onTrue(new InstantCommand(() -> limelightSubsystem.toggleDriverCam()));

    // Log target information to the SmartDashboard
    new Trigger(() -> OI.OPERATOR_CONTROLLER.getButton(btn.Start))
        .onTrue(new InstantCommand(() -> limelightSubsystem.logTargetInfo()));
  }

  private static void configureDriver() {
    // Button 'X' (debounced 0.25s) will reset heading
    final var cRumble = OI.DRIVER_CONTROLLER.getRumbleCommand(0.5, 0.5, 0.25);
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.X))
        .debounce(0.25) // Wait 0.25s to avoid accidental press
        .onTrue(
            new InstantCommand(
                () -> {
                  // Reset odometry so that forward is away from the driver station
                  Drivetrain.resetOdometry(
                      new Pose2d(Drivetrain.getPose().getTranslation(), new Rotation2d(0)));
                  // Rumble to indicate odometry has been reset
                  cRumble.schedule();
                }));

    // Button 'Y' (hold) will set drivetrain to x-stance (for stability)
    final var cXStance = new RunCommand(Drivetrain::toXStance, Drivetrain.getInstance());
    OI.DRIVER_CONTROLLER.onTrue(btn.Y, cXStance::schedule);
    OI.DRIVER_CONTROLLER.onFalse(btn.Y, cXStance::cancel);
  }

  private static void configureOperator() {
    // Set up a binding to run the KitBot intake command while the operator is pressing and holding
    // the left bumper.
    final Command intakeCommand = KitBot.getInstance().getIntakeCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.LB, intakeCommand::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.LB, intakeCommand::cancel);

    // Set up a binding to run the KitBot shoot command while the operator is pressing and holding
    // the right bumper.
    final Command shootCommand = KitBot.getInstance().getShootCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.RB, shootCommand::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.RB, shootCommand::cancel);

    // Set up a binding to run the Amper intake command while the operator is pressing and holding
    // the A button.
    final Command amperIntakeCommand = Amper.getInstance().getIntakeCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.A, amperIntakeCommand::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.A, amperIntakeCommand::cancel);

    // Set up a binding to run the Amper score command while the operator is pressing and holding
    // the Y button.
    final Command amperScoreCommand = Amper.getInstance().getScoreCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.Y, amperScoreCommand::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.Y, amperScoreCommand::cancel);

    // Set up a binding to toggle the Amper hold position command when the operator presses the X
    // button.
    OI.OPERATOR_CONTROLLER.onTrue(
        btn.X,
        () -> {
          if (!Amper.getInstance().isHoldPositionActive) {
            // Start the hold position command.
            Amper.getInstance().getHoldPositionCommand().schedule();
            Amper.getInstance().isHoldPositionActive = true;
          } else {
            // Stop the hold position command.
            Amper.getInstance().stop();
            Amper.getInstance().isHoldPositionActive = false;
          }
        });

    /*
     * The command will schedule when the joystick is first moved and then be active forever.
     * The joystick calls come with a build-in deadband, see the CustomController class.
     *
     * Make the extendClimbers command the default command for the Climbers subsystem so that it's always running. Do this in the constructor.
     * Also, make a private constructor. Even if its empty.
     */

    final Command cExtendClimbers = Climbers.getInstance().extendClimbers();
    // Joystick Y controls climbers
    new Trigger(
            () ->
                Math.abs(OI.OPERATOR_CONTROLLER.getLeftY()) > 0.15
                    || Math.abs(OI.OPERATOR_CONTROLLER.getRightY()) > 0.15)
        .onTrue(new InstantCommand(() -> cExtendClimbers.schedule()));
  }

  // Calls all subsystem stop methods. Does not stop commands.
  public static void stop() {
    for (SafeSubsystem e : SafeSubsystem.getAll()) {
      e.stop();
    }
  }
}
