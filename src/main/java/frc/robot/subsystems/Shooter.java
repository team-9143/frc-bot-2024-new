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

  /** Used to apply tension if a game piece is being held in the intake. */
  private static boolean m_holding = false; // TODO(shooter): this is never set to true. Is it needed? delete otherwise.

  /** Used to invert wheels for intaking */
  private static boolean m_inverted = false;

  private final static CANSparkMax m_motor = new CANSparkMax(ShooterConsts.kShooterID, MotorType.kBrushless);

  private final static TalonFX m_holdingMotor = new TalonFX(0); // TODO(shooter): this variable should be final and follow the naming system used elsewhere (m_holdingMotor or similar). Also, put the ID in ShooterConsts

  private static final RelativeEncoder m_encoder = m_motor.getEncoder(); // TODO(shooter): what is this encoder used for? delete if unnecessary. Also, you need to set conversion factors for the pulleys/gears before you use this data to have accurate units (ratios go into PhysConsts)

  private Shooter() {
    m_encoder.setMeasurementPeriod(20);
    m_encoder.setPosition(0);

    // If inverted and has a game piece, apply tension to hold in a piece
    setDefaultCommand(startEnd(
      () -> {if (m_holding && m_inverted) {m_holdingMotor.setVoltage(ShooterConsts.kHoldVolts);}},
      Shooter::stop
    ));
  }

  // TODO(shooter): if this is a command, it should say so. Add docs. Also, why is this a command? Finally, why is inverted set to false during the duration of the command? And why doesn't it require any subsystem?
  public void invert() {
    m_inverted = m_inverted == true ? false : true;
  }

  public double getSpeed() {return m_motor.get();} // TODO(shooter): Document units. Do we need this method for anything? If we do, it should be returning RPM or RPS measured by encoder

  // TODO(shooter): theres another motor to stop too...
  public static void stop() {
    m_motor.stopMotor();
  }

  /** @return a command to intake a game piece */
  // TODO(shooter): then why is it called getShootCommand?
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_motor.setVoltage(m_inverted ? ShooterConsts.kSourceIntakeVolts : -ShooterConsts.kSourceIntakeVolts * 4); // TODO(shooter): why is this inverted sometimes?
      },
      Shooter::stop
    );
  }

  // TODO(shooter): add documentation, have it stop automatically after fed
  public Command getFeedCommand() {
    return startEnd(
      () -> {
        m_holdingMotor.setVoltage(12); // TODO(shooter): trust me, put this number in ShooterConsts
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