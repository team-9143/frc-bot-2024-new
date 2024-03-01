package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.ShooterConsts;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
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
    private final static CANSparkMax
      m_motor = new CANSparkMax(ShooterConsts.kShooterID, CANSparkLowLevel.MotorType.kBrushless),
      neoMotorWithSparksMax = new CANSparkMax(ShooterConsts.kShooterID, CANSparkLowLevel.MotorType.kBrushless);

    static { neoMotorWithSparksMax.follow(m_motor); }

  private static final RelativeEncoder m_encoder = m_motor.getEncoder();

  private Shooter() {
    m_encoder.setMeasurementPeriod(20);
    m_encoder.setPosition(0);

    // If inverted and has a game piece, apply tension to hold in a piece
    setDefaultCommand(startEnd(
      () -> {if (m_holding && m_inverted) {holdingMotor.setVoltage(ShooterConsts.kHoldVolts);}},
      Shooter::stop
    ));
  }

  public Command invert() {
    return startEnd(() -> m_inverted = false, () -> m_inverted = true);
  }

  public double getSpeed() {return m_motor.get();}

  public static void stop() {
    m_motor.stopMotor();
  }

  /** @return a command to intake a game piece */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_motor.setVoltage(m_inverted ? ShooterConsts.kSourceIntakeVolts : -ShooterConsts.kSourceIntakeVolts * 4);
      },
      Shooter::stop
    );
  }

  public Command getFeedCommand() {
    return startEnd(
      () -> {
        holdingMotor.setVoltage(12);
      },
      Shooter::stop);
  }

  /** @return a command to spit a game piece at partial speed for amp */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_motor.setVoltage(m_inverted ? -ShooterConsts.kSpitVolts : ShooterConsts.kSpitVolts);
        m_holding = false;
      },
      Shooter::stop
    );
  }
}