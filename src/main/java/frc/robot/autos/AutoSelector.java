package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logger.Logger;
import frc.robot.subsystems.Amper;
import java.util.Map;

// TODO: Test 4 escape autos on both alliances
// Contains auto types, choosers, and compiler.
public class AutoSelector {
  private static final MutableChooser<StartPose> chooser_startPose =
      new MutableChooser<>(StartPose.Wing);
  private static final MutableChooser<Starter> chooser_starter = new MutableChooser<>(Starter.None);
  private static final MutableChooser<Body> chooser_body = new MutableChooser<>(Body.None);

  // Initializes shuffleboard choosers for auto.
  public static void init() {
    chooser_startPose.setAll(
        StartPose.Front_Side_V2,
        StartPose.Amp_Side_V4,
        StartPose.Source_Side_V3,
        StartPose.DNU_Source_Side_Short);
    chooser_starter.setAll(Starter.Shoot, Starter.Wait_and_shoot);
    chooser_body.setAll(Body.Escape);

    // Add auto tab to Shuffleboard.
    var autoTab = Shuffleboard.getTab("Auto");
    autoTab.add("Start Pose", chooser_startPose).withPosition(0, 0).withSize(4, 2);
    autoTab.add("Starter", chooser_starter).withPosition(0, 2).withSize(4, 2);
    autoTab.add("Body", chooser_body).withPosition(0, 4).withSize(4, 2);

    // Limelight stream, maybe
    // HttpCamera limelight = new HttpCamera("Limelight", "http://10.91.43.11:5800/stream.mjpg");
    // tab.add(httpCamera).withPosition(4, 0).withSize(15, 7);

    // Add match play tab to Shuffleboard.
    var matchPlayTab = Shuffleboard.getTab("Match Play");
    matchPlayTab
        .addBoolean(
            "Reset Defaults If Red",
            () ->
                !(chooser_startPose.isUpdateReq()
                    || chooser_starter.isUpdateReq()
                    || chooser_body.isUpdateReq()))
        .withPosition(0, 0)
        .withSize(4, 2);
    matchPlayTab
        .addBoolean("Amper: Hold Position", () -> Amper.getInstance().isHoldPositionActive)
        .withPosition(0, 2)
        .withSize(4, 2);
    matchPlayTab
        .addCamera(
            "Driver Camera",
            "USB Camera 0",
            "mjpg:http://roboRIO-9143-FRC.local:1181/?action=stream")
        .withProperties(Map.of("showControls", false))
        .withPosition(4, 0)
        .withSize(15, 6);
  }

  // Returns a full auto routine.
  public static Command getAuto() {
    var startPose = chooser_startPose.getSelected();
    var starter = chooser_starter.getSelected();
    var body = chooser_body.getSelected();

    // Log alliance and auto selection.
    Logger.log((Pathing.isRedAlliance() ? "RED ALLIANCE" : "BLUE ALLIANCE"));
    Logger.log(
        "AUTO: " + startPose.toString() + ", " + starter.toString() + ", " + body.toString());

    return startPose.getCommand().andThen(starter.getCommand()).andThen(body.getCommand(startPose));
  }

  static {
    NamedCommands.registerCommand("Empty", new InstantCommand());
  }
}
