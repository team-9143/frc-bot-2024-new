package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConsts;
import frc.robot.Constants.PhysConsts;
import frc.robot.util.SparkUtils;

public class Climbers extends SafeSubsystem {
  private static final CANSparkMax m_leftClimberMotor =
      new CANSparkMax(ClimberConsts.kLeftClimberID, MotorType.kBrushless);

  static {
    // Follower setup
    var follower = new CANSparkMax(ClimberConsts.kRightClimberID, MotorType.kBrushless);
    SparkUtils.configure(
        follower,
        () -> follower.setIdleMode(IdleMode.kCoast),
        () -> follower.follow(m_leftClimberMotor),
        () -> SparkUtils.setPeriodicFrames(follower, 10, 0, 0, 0, 0, 0, 0));

    // Main motor setup
    SparkUtils.configure(
        m_leftClimberMotor,
        () -> m_leftClimberMotor.setIdleMode(IdleMode.kCoast),
        () -> m_leftClimberMotor.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(m_leftClimberMotor, 10, 0, 0, 0, 0, 0, 0));
  }

  private static final Climbers m_instance = new Climbers();

  /** Returns the singleton instance */
  public static Climbers getInstance() {
    return m_instance;
  }

  public Command extendClimbers() {
    return startEnd(
        () -> m_leftClimberMotor.setVoltage(ClimberConsts.kLeftClimberVolts), this::stop);
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub

  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }
}
