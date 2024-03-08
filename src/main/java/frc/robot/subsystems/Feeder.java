package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConsts;
import frc.robot.logger.Logger;
import java.util.function.Supplier;

/** Controls feeder wheels */
public class Feeder extends SafeSubsystem {
  private static Feeder m_instance;

  /** Returns the singleton instance */
  public static synchronized Feeder getInstance() {
    if (m_instance == null) {
      m_instance = new Feeder();
    }
    return m_instance;
  }

  private static final TalonFXConfiguration configs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  private static final TalonFX m_motor = new TalonFX(FeederConsts.kFeedMotorID);
  private static final Supplier<Double> m_voltageSupplier;

  static {
    m_motor.getConfigurator().apply(configs);
    var voltageSignal = m_motor.getMotorVoltage();
    voltageSignal.setUpdateFrequency(50);
    m_voltageSupplier = voltageSignal.asSupplier();
    m_motor.optimizeBusUtilization();
  }

  private Feeder() {}

  /** Returns a command to feed a note from the intake to the shooter */
  public Command getFeedCommand() {
    return startEnd(() -> m_motor.setVoltage(FeederConsts.kFeedVolts), this::stop);
  }

  /** Returns a command to deeply intake a note from the source */
  public Command getDeepIntakeCommand() {
    return startEnd(() -> m_motor.setVoltage(FeederConsts.kDeepIntakeVolts), this::stop);
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory() + "voltage", m_voltageSupplier.get());
  }
}
