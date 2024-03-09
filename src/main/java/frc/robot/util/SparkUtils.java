package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.logger.Logger;
import java.util.function.Supplier;

/**
 * Utility class for configuration of REV Spark motor controllers. Mostly stolen from Team 1155
 * (thank you Asa)
 */
public class SparkUtils {
  public static final int MAX_ATTEMPTS = 3;
  public static final int FRAME_STRATEGY_DISABLED = 65535;

  /**
   * Formats the name of a spark with its CAN ID.
   *
   * @param spark The spark to find the name of.
   * @return The name of a spark.
   */
  public static String getName(CANSparkBase spark) {
    return "Spark [" + spark.getDeviceId() + "]";
  }

  /**
   * Fully configures a Spark Max/Flex with all provided configs and burns to flash.
   *
   * <p>Each config is applied until success, or until the number of attempts exceed {@link
   * SparkUtils#MAX_ATTEMPTS}.
   *
   * @param spark the spark to configure
   * @param config the configuration to apply
   */
  @SafeVarargs
  public static void configure(CANSparkBase spark, Supplier<REVLibError>... configs) {
    configure(spark, spark::restoreFactoryDefaults, 1);
    configure(spark, () -> spark.setCANTimeout(50), 1);
    configure(spark, spark::clearFaults, 1);
    for (var config : configs) {
      configure(spark, config, 1);
    }
    configure(spark, () -> spark.setCANTimeout(20), 1);

    spark.burnFlash();
    checkErr(spark);
    checkFaults(spark);
  }

  /**
   * Recursively configures a specific value on a spark, until {@code attempt} exceeds {@code
   * MAX_ATTEMPTS}.
   *
   * @param spark the spark to configure
   * @param config the configuration to apply to the spark
   * @param attempt the current attempt number
   */
  private static void configure(CANSparkBase spark, Supplier<REVLibError> config, int attempt) {
    if (attempt > MAX_ATTEMPTS) {
      Logger.reportError(getName(spark) + ": FAILED TO SET PARAMETER");
      return;
    }
    if (attempt > 1) {
      Logger.reportWarning(
          getName(spark) + ": setting parameter failed: " + (attempt - 1) + "/" + MAX_ATTEMPTS);
    }

    REVLibError error = config.get();
    if (error != REVLibError.kOk) {
      Timer.delay(0.01);
      configure(spark, config, attempt + 1);
    }
  }

  /**
   * This is a shorthand for {@link CANSparkBase#setPeriodicFramePeriod()}. Use 0 to represent a
   * disabled frame. Meant to be passed into {@link SparkUtils#configure()}.
   *
   * <p>Status 7 appears to be useless, only used for IAccum.
   *
   * @param spark The spark to configure
   * @param status0 applied output, faults | default 10
   * @param status1 integrated velocity, temperature, input voltage, current | default 20
   * @param status2 integrated position | default 20
   * @param status3 analog encoder | default 50
   * @param status4 alternate quadrature encoder | default 20
   * @param status5 duty cycle position | default 200
   * @param status6 duty cycle velocity | default 200
   * @return {@link REVLibError#kOk}
   * @see https://docs.revrobotics.com/brushless/spark-max/control-interfaces
   */
  public static REVLibError setPeriodicFrames(
      CANSparkBase spark,
      int status0,
      int status1,
      int status2,
      int status3,
      int status4,
      int status5,
      int status6) {
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus0, status0 != 0 ? status0 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus1, status1 != 0 ? status1 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus2, status2 != 0 ? status2 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus3, status3 != 0 ? status3 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus4, status4 != 0 ? status4 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus5, status5 != 0 ? status5 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () ->
            spark.setPeriodicFramePeriod(
                PeriodicFrame.kStatus6, status6 != 0 ? status6 : FRAME_STRATEGY_DISABLED),
        1);
    configure(
        spark,
        () -> spark.setPeriodicFramePeriod(PeriodicFrame.kStatus7, FRAME_STRATEGY_DISABLED),
        1);

    return REVLibError.kOk; // Always returns kOk, errors will be logged by calls to configure()
  }

  /**
   * This is a workaround since {@link CANSparkBase#setInverted(boolean)} does not return a {@code
   * REVLibError} because it is overriding {@link
   * edu.wpi.first.wpilibj.motorcontrol.MotorController}.
   *
   * <p>This call has no effect if the controller is a follower. To invert a follower, see the
   * follow() method.
   *
   * @param spark The spark to set inversion of.
   * @param isInverted The state of inversion, true is inverted.
   * @return {@link REVLibError#kOk} if successful.
   */
  public static REVLibError setInverted(CANSparkBase spark, boolean isInverted) {
    spark.setInverted(isInverted);
    return spark.getLastError();
  }

  /**
   * Checks a spark and reports the last error encountered.
   *
   * @param spark the spark to check
   */
  public static void checkErr(CANSparkBase spark) {
    var err = spark.getLastError();
    if (err != REVLibError.kOk) {
      Logger.reportError(getName(spark) + ": " + err.toString());
    }
  }

  /**
   * Checks a spark and logs the current recorded faults.
   *
   * @param spark the spark to check
   */
  public static void checkFaults(CANSparkBase spark) {
    int faults = spark.getFaults();
    for (FaultID id : FaultID.values()) {
      if ((faults & (1 << id.value)) > 0) {
        Logger.reportError(getName(spark) + ": fault " + id.toString());
      }
    }
  }
}
