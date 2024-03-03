package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConsts;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;

public class ShooterShoot extends Command {
  private static final Shooter sShooter = Shooter.getInstance();
  private static final Set<Subsystem> m_requirements = Set.of(sShooter);

  @Override
  public void initialize(){}

  @Override
  public void execute() {
    Shooter.m_ShooterBottom.setVoltage(ShooterConsts.kShootVolts);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public Set<Subsystem> getRequirements() {
    return m_requirements;
  }
}