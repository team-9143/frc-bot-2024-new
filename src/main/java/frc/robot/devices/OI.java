package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.DeviceConsts;

public class OI {
  public static final Controller DRIVER_CONTROLLER = new Controller(DeviceConsts.kDriverPort);
  public static final Controller OPERATOR_CONTROLLER = new Controller(DeviceConsts.kOperatorPort);

  public static final Limelight LIMELIGHT =
      new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));
}
