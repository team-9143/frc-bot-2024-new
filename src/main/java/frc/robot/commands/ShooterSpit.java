package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConsts;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;

public class ShooterSpit extends Command {
  private static final Shooter sShooter = Shooter.getInstance(); // TODO(shooter): What is this variable for? Is it needed?
  private static final Set<Subsystem> m_requirements = Set.of(sShooter);

  @Override
  public void initialize(){}

  // TODO(shooter): Move to `initialize`, only needs to be called once
  @Override
  public void execute() {
    Shooter.m_ShooterBottom.setVoltage(ShooterConsts.kSpitVolts);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // TODO(shooter): add Shooter.stop() on end, or make the Shooter subsystem's default command call it
  @Override
  public void end(boolean interrupted) {}

  @Override
  public Set<Subsystem> getRequirements() {
    return m_requirements;
  }
}