package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConsts;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

// TODO(shooter): TLDR theres a lot of stuff here that is clearly unneded or implemented wrong, make sure you understand eveyrthing in here and can explain why its there and when its used.

// TODO(shooter): this should extend SafeSubsystem. see ExampleSubsystem to see how to implement relevant methods

// TODO(shooter): Why are we making commands files instead of using the concise inline ones? They are much easier to set up/read and that way, your motor variables aren't public (good practice to have them protected from unknown calls)

// TODO(shooter): Consider making this motor its own "Feeder" subsystem because it isn't directly connected to the shooter (would be much easier to code with after the intake and pivot are added), and would allow for commands to control the shooter wheels and feeder wheels individually (different subsystem req's)
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

  public final static CANSparkMax m_ShooterBottom = new CANSparkMax(ShooterConsts.kBottomShooterMotorID, MotorType.kBrushless);

  public final static CANSparkMax m_ShooterTop = new CANSparkMax(ShooterConsts.kTopShooterMotorID, MotorType.kBrushless);

  public final static TalonFX m_feedMotor = new TalonFX(ShooterConsts.kFeedMotorID);

  // TODO(shooter) If its for intaking, why is it called getShootCommand? Make two separate commands for source intaking and shooting, and use separate constants for the speed of both.
  /** @return a command to intake a game piece using shooter wheels (only while we don't have an intake) */
  public Command getShootCommand() {
    return startEnd(
      () -> {
        m_ShooterBottom.setVoltage(-ShooterConsts.kSourceIntakeVolts); // ability to invert for source intake
        m_ShooterTop.setVoltage(-ShooterConsts.kSourceIntakeVolts);
      },
      Shooter::stop
    );
  }

  // TODO(shooter): this documentation is confusing, say something like `@return a command to feed a note from the intake to the shooter`
  /** Uses bottom shooter motor to feed the top shooter motors */
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
        m_ShooterBottom.setVoltage(ShooterConsts.kSpitVolts);

      },
      Shooter::stop
    );
  }

  public static void stop() {
    m_ShooterBottom.stopMotor();
    m_ShooterTop.stopMotor();
    m_feedMotor.stopMotor();
  }
}