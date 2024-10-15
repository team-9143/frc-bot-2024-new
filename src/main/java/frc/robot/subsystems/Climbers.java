package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.devices.OI;
import frc.robot.util.SparkUtils;

public class Climbers extends SafeSubsystem {
  private static final CANSparkMax m_leftClimberMotor =
      new CANSparkMax(ClimberConstants.kLeftClimberID, MotorType.kBrushless);
  private static final CANSparkMax m_rightClimberMotor =
      new CANSparkMax(ClimberConstants.kRightClimberID, MotorType.kBrushless);

  static {
    SparkUtils.configure(
        m_leftClimberMotor,
        () -> m_leftClimberMotor.setIdleMode(IdleMode.kBrake),
        () -> m_leftClimberMotor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(m_leftClimberMotor, 10, 0, 0, 0, 0, 0, 0));

    SparkUtils.configure(
        m_rightClimberMotor,
        () -> m_rightClimberMotor.setIdleMode(IdleMode.kBrake),
        () -> m_rightClimberMotor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(m_rightClimberMotor, 10, 0, 0, 0, 0, 0, 0));
  }

  private static final Climbers m_instance = new Climbers();

  // Returns the singleton instance
  public static Climbers getInstance() {
    return m_instance;
  }

  public Command extendClimbers() {
    return runEnd(
        () -> {
          m_rightClimberMotor.setVoltage(
              (ClimberConstants.kClimberSpeed * 12) * -OI.OPERATOR_CONTROLLER.getRightY());
          m_leftClimberMotor.setVoltage(
              (ClimberConstants.kClimberSpeed * 12) * -OI.OPERATOR_CONTROLLER.getLeftY());
        },
        this::stop);
  }

  @Override
  public void log() {}

  @Override
  public void stop() {
    m_leftClimberMotor.stopMotor();
    m_rightClimberMotor.stopMotor();
  }
}
