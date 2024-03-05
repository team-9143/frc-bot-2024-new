package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConsts;
import frc.robot.Constants.ShooterConsts;

import com.ctre.phoenix6.hardware.TalonFX;

public class Feeder extends SafeSubsystem {
    private static Feeder m_instance;

  /** @return the singleton instance */
  public static synchronized Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  public final static TalonFX m_feedMotor = new TalonFX(ShooterConsts.kFeedMotorID);


      /** @return a command to feed a note from the intake to the shooter */
  public Command getFeedCommand() {
    return startEnd(
      () -> {
        m_feedMotor.setVoltage(FeederConsts.kHoldingVolts);
      },
      Shooter.getInstance()::stop);
  }

  @Override
  public void stop() {
    m_feedMotor.stopMotor();
  }
    @Override
    public void log() {
        // TODO Auto-generated method stub
        

    }
}