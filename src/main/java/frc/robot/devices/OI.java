package frc.robot.devices;

import frc.robot.Constants.DeviceConstants;

public class OI {
  public static final Controller DRIVER_CONTROLLER = new Controller(DeviceConstants.kDriverPort);
  public static final Controller OPERATOR_CONTROLLER =
      new Controller(DeviceConstants.kOperatorPort);

  // public static final Limelight LIMELIGHT =
  // new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));
}
