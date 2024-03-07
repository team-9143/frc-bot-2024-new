package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConsts;

import com.ctre.phoenix6.hardware.TalonFX;

/** Controls feeder wheels */
public class Feeder extends SafeSubsystem {
    private static Feeder m_instance;

  /** @return the singleton instance */
  public static synchronized Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  private final static TalonFX m_feedMotor = new TalonFX(FeederConsts.kFeedMotorID);
  static {
    // TODO(shooter): Extra - use the talon configurator to set current limits and neutral mode here.
    m_feedMotor.optimizeBusUtilization();
  }

  private Feeder() {}

  // TODO(shooter): You have to actually set this variable somewhere (hint - when the source intake and shoot commands start). Also, forgetting "public static" - alternatively, make "private static" and have a setter method (this would actually be better for debugging and code maintanence purposes). I'd prefer static for this field simply because there's no need for it to not be.
  private static boolean m_holding = false;

  /** Sets whether the robot indexer is holding a game piece. */
  public static void setHolding(boolean holding) {
    m_holding = holding;
  }

  public void initDefaultCommand() {
    setDefaultCommand(runEnd(
      () -> m_feedMotor.setVoltage(m_holding ? FeederConsts.kFeedVolts : 0),
      this::stop
    ));
  }
  /** @return a command to feed a note from the intake to the shooter */
  public Command getFeedCommand() {
    return startEnd(
      () -> m_feedMotor.setVoltage(FeederConsts.kFeedVolts),
      this::stop
    );
  }

  @Override
  public void stop() {
    m_feedMotor.stopMotor();
  }

  @Override
  public void log() {}
}