// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.autos.AutoSelector;
import frc.robot.logger.Logger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TunableNumber;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // This function is run when the robot is first started up and should be used for any
  // initialization code.

  // private CANdle candle; // CANdle object for controlling LEDs
  // private boolean ledOn = true; // Tracks the LED on/off state
  // private GenericEntry ledToggleEntry; // Entry in Shuffleboard for the toggle button

  // Define the GRB color (for example, red)
  // private final int[] color = {255, 0, 0}; // Set to any GRB color you want

  @Override
  public void robotInit() {
    RobotContainer.init();

    // Initialize the CANdle
    CANdle candle = new CANdle(3); // Set the correct CAN ID for the CANdle

    // Configure CANdle settings if necessary
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);

    candle.setLEDs(255, 255, 255);

    // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 60);
    // candle.animate(rainbowAnim);

    /*
    // Add a toggle button on Shuffleboard
    ledToggleEntry =
        Shuffleboard.getTab("LED Control") // Create a new tab called "LED Control"
            .add("LED Toggle", ledOn) // Add a toggle entry
            .withWidget("Toggle Button") // Use a toggle button widget
            .getEntry();
    */

    AutoSelector.init();
    // Limelight.init();

    CameraServer.startAutomaticCapture();

    // Add periodic callback for drivetrain updates
    addPeriodic(
        Drivetrain::update,
        DriveConstants.kPeriodMs / 1000d,
        (kDefaultPeriod - (DriveConstants.kPeriodMs / 1000d)) / 2);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateLogs();
  }

  @Override
  public void driverStationConnected() {
    RobotContainer.initDS();
  }

  @Override
  public void autonomousInit() {
    AutoSelector.getAuto().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    /*
    // Check if the toggle button has been changed in Shuffleboard
    boolean toggle = ledToggleEntry.getBoolean(true);

    // If the button state has changed, update LED state
    if (toggle != ledOn) {
      ledOn = toggle; // Update the LED state

      if (ledOn) {
        // Turn the LEDs on to the specified color
        candle.setLEDs(color[0], color[1], color[2]);
      } else {
        // Turn the LEDs off (black color)
        setLEDColor(0, 0, 0);
      }

      // Update the toggle state in Shuffleboard
      ledToggleEntry.setBoolean(ledOn);
    }
    */
  }

  float Kp = -0.1f;
  float min_command = 0.05f;

  /*
  std::shared_ptr<NetworkTable> table = NetworkTable:: GetTable("limelight");
   float tx = table->GetNumber("tx");

    if (joystick->GetRawButton(9))
  {
  	float heading_error = -tx;
  	float steering_adjust = 0.0f;
  	if (Math.abs(heading_error) > 1.0)
  	{
  		if (heading_error < 0)
  		{
  			steering_adjust = Kp*heading_error + min_command;
  		}
  		else
  		{
  			steering_adjust = Kp*heading_error - min_command;
  		}
  	}

  	left_command += steering_adjust;
  	right_command -= steering_adjust;
  }
  */

  @Override
  public void disabledInit() {
    RobotContainer.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    // Enable commands in test mode
    CommandScheduler.getInstance().enable();

    // Initialize Tunables
    TunableNumber.initializeShuffleboard();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // Update all tunable numbers
    TunableNumber.updateAll();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  /*
  // Helper method to set the LED color
  public void setLEDColor(int g, int r, int b) {
    candle.setLEDs(g, r, b); // Sets LEDs to the specified RGB color
  }
  */
}
