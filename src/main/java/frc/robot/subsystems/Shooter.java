package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConsts;
import frc.robot.Constants.PhysConsts;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/** Controls indexer wheels. */
public class Shooter extends SubsystemBase {
  private static Shooter m_instance;

  /** @return the singleton instance */
  public static synchronized Shooter getInstance() {
    if (m_instance == null) {
      m_instance = new Shooter();
    }
    return m_instance;
  }

  TalonFX holdingMotor = new TalonFX(0);
  
  /** Used to apply tension if a game piece is being held in the intake. */
  private static boolean m_holding = false;

  /** Used to invert wheels */
  private static boolean m_inverted = false;

  private static final CANSparkMax m_motor = new CANSparkMax(DeviceConsts.kShooterID, MotorType.kBrushless);

  private static final CANSparkMax neoMotorWithSparksMax = new CANSparkMax(DeviceConsts.kShooterID, MotorType.kBrushless);

  private static final RelativeEncoder m_encoder = m_motor.getEncoder();

  private Shooter() {
    m_encoder.setPositionConversionFactor(PhysConstants.kTiltGearbox); // UNIT: rotations
    m_encoder.setVelocityConversionFactor(PhysConstants.kTiltGearbox); // UNIT: rpm
    m_encoder.setMeasurementPeriod(20);
    m_encoder.setPosition(0);

    // If inverted and has a game piece, apply tension to hold in a piece
    setDefaultCommand(startEnd(
      () -> {if (m_holding && m_inverted) {holdingMotor.set(IntakeConstants.kHoldSpeed);}},
      Shooter::stop
    ));
  }

  public double getSpeed() {return m_motor.get();}

  public static void stop() {
    m_motor.stopMotor();
  }
                                 
  /** @return a command to intake a game piece */
  public Command getIntakeCommand() {
    return startEnd(
      () -> {
        m_motor.set(m_inverted ? -PhysConsts.kNEOMaxVoltage : PhysConsts.kNEOCurrentLimit);
        m_holding = true;
      },
      Shooter::stop
    );
  }

  /** @return a command to shoot a game piece at full speed for shooter */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_motor.set(m_inverted ? -IntakeConstants.kShootSpeed : IntakeConstants.kShootSpeed);
        m_holding = false;
      },
      Shooter::stop
    );
  }

  /** @return a command to spit a game piece at partial speed for amp */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_motor.set(m_inverted ? -IntakeConstants.kSpitSpeed : IntakeConstants.kSpitSpeed);
        m_holding = false;
      },
      Shooter::stop
    );
  }
}