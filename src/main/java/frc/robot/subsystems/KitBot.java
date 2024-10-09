// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.KitBotConstants;
import frc.robot.util.SparkUtils;

// Controls shooter wheels.
public class KitBot extends SafeSubsystem {
  private static final CANSparkMax feeder_motor =
      new CANSparkMax(KitBotConstants.kFeederID, MotorType.kBrushed);
  private static final CANSparkMax launcher_motor =
      new CANSparkMax(KitBotConstants.kLauncherID, MotorType.kBrushed);

  static {
    // Feeder motor setup
    SparkUtils.configure(
        feeder_motor,
        () -> feeder_motor.setIdleMode(IdleMode.kBrake),
        () -> feeder_motor.setSmartCurrentLimit(KitBotConstants.kFeedCurrentLimit));

    // Launcher motor setup
    SparkUtils.configure(
        launcher_motor,
        () -> launcher_motor.setIdleMode(IdleMode.kBrake),
        () -> launcher_motor.setSmartCurrentLimit(KitBotConstants.kLauncherCurrentLimit));
  }

  private static final KitBot m_kitbot = new KitBot();

  // Returns the singleton instance
  public static KitBot getInstance() {
    return m_kitbot;
  }

  // Returns a command to intake a game piece using launcher wheels
  public Command getIntakeCommand() {
    return new ParallelCommandGroup(
        startEnd(
            () -> feeder_motor.setVoltage(KitBotConstants.kIntakeFeederSpeed * 12),
            () -> feeder_motor.stopMotor()),
        startEnd(
            () -> launcher_motor.setVoltage(KitBotConstants.kIntakeLauncherSpeed * 12),
            () -> launcher_motor.stopMotor()));
  }

  // Returns a command to shoot a game piece using launcher wheels.
  public Command getShootCommand() {
    return new ParallelCommandGroup(
        // Start launcher motor immediately.
        startEnd(
            () -> launcher_motor.setVoltage(KitBotConstants.kLauncherSpeed * 12),
            () -> launcher_motor.stopMotor()),

        // Start feeder motor after a delay.
        Commands.sequence(
            Commands.waitSeconds(KitBotConstants.kFeederDelay),
            startEnd(
                () -> feeder_motor.setVoltage(KitBotConstants.kFeederSpeed * 12),
                () -> feeder_motor.stopMotor())));
  }

  @Override
  public void stop() {
    feeder_motor.stopMotor();
    launcher_motor.stopMotor();
  }

  @Override
  public void log() {}
}
