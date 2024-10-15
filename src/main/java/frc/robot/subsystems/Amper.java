// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmperConstants;
import frc.robot.util.SparkUtils;

public class Amper extends SafeSubsystem {

  private static final CANSparkMax amper_motor =
      new CANSparkMax(AmperConstants.kAmperID, MotorType.kBrushed);

  static {
    // Amper motor setup
    SparkUtils.configure(
        amper_motor,
        () -> amper_motor.setIdleMode(IdleMode.kBrake),
        () -> amper_motor.setSmartCurrentLimit(AmperConstants.kAmperCurrentLimit));
  }

  private static final Amper m_amper = new Amper();

  // Returns the singleton instance
  public static Amper getInstance() {
    return m_amper;
  }

  public boolean isHoldPositionActive = false;

  // Method to set the motor for intake mode.
  public Command getIntakeCommand() {
    return startEnd(
        () -> amper_motor.setVoltage(AmperConstants.kAmperIntakeSpeed * 12),
        () -> amper_motor.stopMotor());
  }

  // Method to set the motot for scoring mode.
  public Command getScoreCommand() {
    return startEnd(
        () -> amper_motor.setVoltage(AmperConstants.kAmperScoreSpeed * 12),
        () -> amper_motor.stopMotor());
  }

  // Method to set the motor at stall speed.
  public Command getHoldPositionCommand() {
    return startEnd(
        () -> amper_motor.setVoltage(AmperConstants.kAmperHoldPositionSpeed * 12),
        () -> amper_motor.stopMotor());
  }

  // Method to stop the motor
  @Override
  public void stop() {
    amper_motor.stopMotor();
  }

  @Override
  public void log() {}
}
