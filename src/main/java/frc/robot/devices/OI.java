package frc.robot.devices;

import frc.robot.Constants.DeviceConsts;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OI {
  public final static Controller DRIVER_CONTROLLER = new Controller(DeviceConsts.kDriverPort);
  public final static Controller OPERATOR_CONTROLLER = new Controller(DeviceConsts.kOperatorPort);

  public final static Limelight LIMELIGHT = new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));
}