package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConsts;
import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.ShooterConsts;
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
    m_encoder.setMeasurementPeriod(20);
    m_encoder.setPosition(0);

    // If inverted and has a game piece, apply tension to hold in a piece
    setDefaultCommand(startEnd(
      () -> {if (m_holding && m_inverted) {holdingMotor.set(ShooterConsts.kHoldSpeed);}},
      Shooter::stop
    ));
  }

  public double getSpeed() {return m_motor.get();}

  public static void stop() {
    m_motor.stopMotor();
  }
                                 
  /** @return a command to intake a game piece */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_motor.setVoltage(m_inverted ? ShooterConsts.kNEOIntakeVoltage : -ShooterConsts.kNEOIntakeVoltage * 4);
      },
      Shooter::stop
    );
  }

  /** @return a command to spit a game piece at partial speed for amp */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_motor.set(m_inverted ? -ShooterConsts.kSpitSpeed : ShooterConsts.kSpitSpeed);
        m_holding = false;
      },
      Shooter::stop
    );
  }
}