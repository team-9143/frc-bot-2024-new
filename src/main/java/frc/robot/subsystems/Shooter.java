package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConsts;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// TODO(shooter): TLDR theres a lot of stuff here that is clearly unneded or implemented wrong, make sure you understand eveyrthing in here and can explain why its there and when its used.

// TODO(shooter): this should extend SafeSubsystem. see ExampleSubsystem to see how to implement relevant methods
/** Controls indexer and shooter wheels. */
public class Shooter extends SubsystemBase {
  private static Shooter m_instance;

  /** @return the singleton instance */
  public static synchronized Shooter getInstance() {
    if (m_instance == null) {
      m_instance = new Shooter();
    }
    return m_instance;
  }

  /** Used to invert wheels for intaking */
  private static boolean m_inverted = false;

  private final static CANSparkMax m_motor = new CANSparkMax(ShooterConsts.kShooterID, MotorType.kBrushless);

  private final static CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConsts.kMotorID, MotorType.kBrushless);

  private final static TalonFX m_feedMotor = new TalonFX(ShooterConsts.kFeedID);
  
  private static final RelativeEncoder m_encoder = m_motor.getEncoder(); // TODO(shooter): what is this encoder used for? delete if unnecessary. Also, you need to set conversion factors for the pulleys/gears before you use this data to have accurate units (ratios go into PhysConsts)

  private Shooter() {
    m_encoder.setMeasurementPeriod(20);
    m_encoder.setPosition(0);
  }

  /**Reverses current value of m_inverted**/
  public void invert() {
    m_inverted = m_inverted == false ? true : false;
  }

  public static void stop() {
    m_motor.stopMotor();
    m_shooterMotor.stopMotor();
  }

  /** @return a command to intake a game piece using shooter wheels (only while we don't have an intake) */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_motor.setVoltage(m_inverted ? ShooterConsts.kSourceIntakeVolts : -ShooterConsts.kSourceIntakeVolts * 4); // ability to invert for source intake
      },
      Shooter::stop
    );
  }

  /**Uses bottom shooter motor to feed the top shooter motors**/
  public Command getFeedCommand() {
    return startEnd(
      () -> {
        m_feedMotor.setVoltage(ShooterConsts.kHoldingVolts); 
      },
      Shooter::stop);
  }

  /** @return a command to spit a game piece at partial speed for amp */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_motor.setVoltage(m_inverted ? -ShooterConsts.kSpitVolts : ShooterConsts.kSpitVolts);
  
      },
      Shooter::stop
    );
  }
}