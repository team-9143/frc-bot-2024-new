// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.commands;

import static frc.robot.Constants.AmperConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreAmp extends Command {

  private final CANSparkMax amperMotor;
  private final boolean intake;
  private final boolean score;

  // Constructor to choose between intake and score modes.
  public ScoreAmp(boolean intake, boolean score) {
    this.amperMotor = new CANSparkMax(kAmperID, MotorType.kBrushed);
    this.intake = intake;
    this.score = score;

    // Set amper motor current limit to avoid burning it out.
    amperMotor.setSmartCurrentLimit(kAmperCurrentLimit);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake) {
      // Intake mode; motor runs in reverse to bring the note in.
      amperMotor.set(kAmperIntakeSpeed);
    } else if (score) {
      // Scoring mode; motor runs forward to release the note.
      amperMotor.set(kAmperScoreSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the note is held, run the motor at stall speed to hold it in place.
    if (!intake && !score) {
      amperMotor.set(kAmperHoldPositionSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command finishes.
    amperMotor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Keep running unitl manually stopped or interrupted.
  }
}
*/
