package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

// TODO: Test autos and force chooser enums to have a getCommand method
/** Contains auto types, choosers, and compiler. */
public class AutoSelector {
  private static final MutableChooser<StartPose> chooser_startPose =
      new MutableChooser<>(StartPose.Wing);
  private static final MutableChooser<Starter> chooser_starter = new MutableChooser<>(Starter.None);
  private static final MutableChooser<Body> chooser_body = new MutableChooser<>(Body.None);

  /** Initializes shuffleboard choosers for auton */
  public static void init() {
    chooser_startPose.setAll(StartPose.Front, StartPose.Inner, StartPose.Outer);
    chooser_starter.setAll(Starter.Shoot, Starter.WaitToShoot);
    chooser_body.setAll(Body.MoveBack);

    chooser_startPose.bindTo(
        (t, u) -> {
          if (u == StartPose.Wing) {
            chooser_body.setAll(Body.MoveBack);
          } else {
            chooser_body.setAll(Body.EscapeSubwoofer);
          }
        });

    // Add to shuffleboard
    var tab = Shuffleboard.getTab("Auton");
    tab.add(chooser_startPose);
    tab.add(chooser_starter);
    tab.add(chooser_body);
    tab.addBoolean(
        "Reset needed",
        () ->
            chooser_startPose.isUpdateReq()
                || chooser_starter.isUpdateReq()
                || chooser_body.isUpdateReq());
  }

  /** Returns a full auto routine */
  public static Command getAuto() {
    var startPose = chooser_startPose.getSelected();
    return new InstantCommand(() -> Drivetrain.resetOdometry(startPose.getPose()))
        .andThen(chooser_starter.getSelected().getAuto())
        .andThen(chooser_body.getSelected().getAuto(startPose));
  }

  static {
    NamedCommands.registerCommand("Empty", new InstantCommand());
  }
}
