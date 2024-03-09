package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logger.Logger;

// TODO: Test autos
/** Contains auto types, choosers, and compiler. */
public class AutoSelector {
  private static final MutableChooser<StartPose> chooser_startPose =
      new MutableChooser<>(StartPose.Wing);
  private static final MutableChooser<Starter> chooser_starter = new MutableChooser<>(Starter.None);
  private static final MutableChooser<Body> chooser_body = new MutableChooser<>(Body.None);

  /** Initializes shuffleboard choosers for auton */
  public static void init() {
    chooser_startPose.setAll(
        StartPose.SubwooferFront, StartPose.SubwooferSource, StartPose.SubwooferAmp);
    chooser_starter.setAll(Starter.Shoot, Starter.WaitToShoot);
    chooser_body.setAll(Body.Escape);

    // Add to shuffleboard
    var tab = Shuffleboard.getTab("Auton");
    tab.add("Start pose", chooser_startPose).withPosition(0, 2).withSize(3, 2);
    tab.add("Starter", chooser_starter).withPosition(3, 2).withSize(3, 2);
    tab.add("Body", chooser_body).withPosition(6, 2).withSize(3, 2);

    tab.addBoolean(
            "Reset needed",
            () ->
                chooser_startPose.isUpdateReq()
                    || chooser_starter.isUpdateReq()
                    || chooser_body.isUpdateReq())
        .withPosition(0, 0)
        .withSize(2, 2);
  }

  /** Returns a full auto routine */
  public static Command getAuto() {
    var startPose = chooser_startPose.getSelected();
    var starter = chooser_starter.getSelected();
    var body = chooser_body.getSelected();

    Logger.log((Pathing.isRedAlliance() ? "RED ALLIANCE" : "BLUE ALLIANCE"));
    Logger.log("AUTON: " + startPose.toString() + ", " + starter.toString() + ", " + body.toString());

    return startPose.getCommand().andThen(starter.getCommand()).andThen(body.getCommand(startPose));
  }

  static {
    NamedCommands.registerCommand("Empty", new InstantCommand());
  }
}
