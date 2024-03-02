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

  // TODO(shooter): You shouldn't need this variable. Have a separate command for intaking and a separate one for shooting.
  /** Used to invert wheels for intaking */
  private static boolean m_inverted = false;

  // TODO(shooter): ID's and names for these first two motors don't match up. Also, how about ShooterTop and ShooterBottom for both? You should be able to set one to follow the other (but inverted) b/c I doubt we'll need more control than that
  private final static CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);

  private final static CANSparkMax m_ShooterTop = new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless);

  private final static TalonFX m_feedMotor = new TalonFX(ShooterConsts.kFeedMotorID);

  private static final RelativeEncoder m_encoder = m_ShooterBottom.getEncoder(); // TODO(shooter): what is this encoder used for? delete if unnecessary. Also, you need to set conversion factors for the pulleys/gears before you use this data to have accurate units (ratios go into PhysConsts)

  private Shooter() {
    m_encoder.setMeasurementPeriod(20);
    m_encoder.setPosition(0);
  }

  // TODO(shooter): Change this to take in a boolean and set the invert variable to that, unless there's a specific reason that this would not be optimal.
  /** Reverses current value of m_inverted */
  public void invert() {
    m_inverted = m_inverted == false ? true : false; // TODO(shooter): Change to `m_inverted = !m_inverted` or `m_inverted ^= true`. Horrendous ternary operator here
  }


  public static void stop() {
    m_ShooterBottom.stopMotor();
    m_ShooterTop.stopMotor();
    m_feedMotor.stopMotor();
  }

  // TODO(shooter) If its for intaking, why is it called getShootCommand? Make two separate commands for source intaking and shooting, and use separate constants for the speed of both.
  /** @return a command to intake a game piece using shooter wheels (only while we don't have an intake) */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(m_inverted ? ShooterConsts.kSourceIntakeVolts : -ShooterConsts.kSourceIntakeVolts * 4); // ability to invert for source intake
      },
      Shooter::stop
    );
  }

  // TODO(shooter): this documentation is confusing, say something like `@return a command to feed a note from the intake to the shooter`. Could also consider making this motor its own "Feeder" subsystem because it isn't directly connected to the shooter (would be much easier to code with after the intake and pivot are added)
  /** Uses bottom shooter motor to feed the top shooter motors */
  public Command getFeedCommand() {
    return startEnd(
      () -> {
        m_feedMotor.setVoltage(ShooterConsts.kHoldingVolts);
      },
      Shooter::stop);
  }

  // TODO(shooter): Why would this ever be inverted if it's for spitting?
  /** @return a command to spit a game piece at partial speed for amp */
  public Command getSpitCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(m_inverted ? -ShooterConsts.kSpitVolts : ShooterConsts.kSpitVolts);

      },
      Shooter::stop
    );
  }
}