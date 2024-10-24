// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  @Override
  public void robotInit() {
    RobotContainer.init();
    AutoSelector.init();

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
  public void teleopPeriodic() {}

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
}
