// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        () -> feeder_motor.setIdleMode(IdleMode.kCoast),
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

  // Returns a command to intake a game piece using both wheels.
  public Command getIntakeCommand() {
    return startEnd(
        () -> {
          feeder_motor.set(KitBotConstants.kIntakeFeederSpeed);
          launcher_motor.set((KitBotConstants.kIntakeLauncherSpeed));
        },
        () -> {
          feeder_motor.stopMotor();
          launcher_motor.stopMotor();
        });
  }

  // Returns a command to shoot a game piece using both wheels.
  public Command getShootCommand() {
    return new SequentialCommandGroup(
        Commands.runOnce(() -> launcher_motor.set(KitBotConstants.kShootLauncherSpeed)),
        Commands.waitSeconds(KitBotConstants.kFeederDelay),
        startEnd(
            () -> {
              launcher_motor.set(KitBotConstants.kShootLauncherSpeed);
              feeder_motor.set(KitBotConstants.kShootFeederSpeed);
            },
            () -> {
              launcher_motor.stopMotor();
              feeder_motor.stopMotor();
            }));
  }

  @Override
  public void stop() {
    feeder_motor.stopMotor();
    launcher_motor.stopMotor();
  }

  @Override
  public void log() {}
}
