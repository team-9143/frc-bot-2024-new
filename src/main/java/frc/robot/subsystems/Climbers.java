package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConsts;
import frc.robot.Constants.PhysConsts;
import frc.robot.devices.OI;
import frc.robot.util.SparkUtils;

// TODO(climbers): Time-allowing, might want to use encoders to stop the motors moving too far
public class Climbers extends SafeSubsystem {
  private static final CANSparkMax m_leftClimberMotor =
      new CANSparkMax(ClimberConsts.kLeftClimberID, MotorType.kBrushless);
  private static final CANSparkMax m_rightClimberMotor =
      new CANSparkMax(ClimberConsts.kRightClimberID, MotorType.kBrushless);

  static {
    // TODO(climbers): Seems clear this will be unused (and is still avail in shooter), can delete
    // Follower setup
    // var follower = new CANSparkMax(ClimberConsts.kRightClimberID, MotorType.kBrushless);
    // SparkUtils.configure(
    //     follower,
    //     () -> follower.setIdleMode(IdleMode.kCoast),
    //     () -> follower.follow(m_leftClimberMotor),
    //     () -> SparkUtils.setPeriodicFrames(follower, 10, 0, 0, 0, 0, 0, 0));

    // TODO(climbers): See if these current limits can be lowered, I don't think they'll need >25
    // Main motor setup
    SparkUtils.configure(
        m_leftClimberMotor,
        () -> m_leftClimberMotor.setIdleMode(IdleMode.kBrake),
        () -> m_leftClimberMotor.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(m_leftClimberMotor, 10, 0, 0, 0, 0, 0, 0));

    SparkUtils.configure(
        m_rightClimberMotor,
        () -> m_rightClimberMotor.setIdleMode(IdleMode.kBrake),
        () -> m_rightClimberMotor.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(m_rightClimberMotor, 10, 0, 0, 0, 0, 0, 0));
  }

  private static final Climbers m_instance = new Climbers();

  /** Returns the singleton instance */
  public static Climbers getInstance() {
    return m_instance;
  }

  // TODO(climbers): Remember to add a private constructor here, even if empty

  // TODO(climbers): The below 2 methods seem unused, delete?

  // TODO(climbers): Try to add a small bit of documentation here! See shooter
  public Command extendClimberLeft() {
    // TODO(climbers): If you want this to update with the joystick, use runEnd()
    return runEnd(
        () ->
            m_leftClimberMotor.setVoltage(
                ClimberConsts.kClimberVolts * -OI.OPERATOR_CONTROLLER.getLeftY()),
        this::stopLeft);
  }

  // TODO(climbers): Try to add a small bit of documentation here! See shooter
  public Command extendClimberRight() {
    // TODO(climbers): If you want this to update with the joystick, use runEnd()
    return runEnd(
        () ->
            m_rightClimberMotor.setVoltage(
                ClimberConsts.kClimberVolts * -OI.OPERATOR_CONTROLLER.getRightY()),
        this::stopRight);
  }

  // TODO(climbers): rename extendClimbers()
  public Command extendClimber() {
    return runEnd(
        () -> {
            m_rightClimberMotor.setVoltage(
                ClimberConsts.kClimberVolts * -OI.OPERATOR_CONTROLLER.getRightY());
            m_leftClimberMotor.setVoltage(
              ClimberConsts.kClimberVolts * -OI.OPERATOR_CONTROLLER.getLeftY());},
        // TODO(climbers): hmmmmm, somethings wrong in the next line
        this::stopRight);
  }

  @Override
  public void log() {}

  @Override
  public void stop() {
    m_leftClimberMotor.stopMotor();
    m_rightClimberMotor.stopMotor();
  }

  // TODO(climbers): These next two methods are unused, delete?

  private void stopRight() {
    m_rightClimberMotor.stopMotor();
  }

  private void stopLeft() {
    m_leftClimberMotor.stopMotor();
  }
}
