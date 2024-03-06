package frc.robot.util;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.SwerveConsts;
import java.util.function.Supplier;

/** Controls a single swerve module. */
public class SwerveModule {
  private static final MagnetSensorConfigs cancoder_config =
      new MagnetSensorConfigs()
          .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
          .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

  private final CANSparkMax drive_motor;
  private final CANSparkMax azimuth_motor;

  private final RelativeEncoder drive_encoder;
  private final CANcoder cancoder;
  private final Supplier<Double> rotationSupplier;

  private final PIDController azimuth_controller;

  private final TunableNumber kS;
  private final TunableNumber kP;
  private final TunableNumber kD;

  /**
   * Creates a swerve module controlled by NEO V1.1 motors, Spark MAX's, and a CANcoder.
   *
   * @param constants module configuration
   * @param period_ms update rate in milliseconds (limit to [8...64] to fully interface with NEO
   *     hall sensor)
   */
  protected SwerveModule(SwerveModuleConstants constants, int period_ms) {
    // Configure drive motor and encoder
    drive_motor = new CANSparkMax(constants.drive_ID, MotorType.kBrushless);
    drive_encoder = drive_motor.getEncoder();
    SparkUtils.configure(
        drive_motor,
        () -> drive_motor.setIdleMode(IdleMode.kBrake),
        () -> drive_motor.setSmartCurrentLimit(PhysConsts.kNEOCurrentLimit),
        () ->
            drive_encoder.setPositionConversionFactor(
                PhysConsts.kSwerveDriveGearbox
                    * PhysConsts.kSwerveWheelCircumferenceMeters), // UNIT: meters
        () ->
            drive_encoder.setVelocityConversionFactor(
                PhysConsts.kSwerveDriveGearbox
                    * PhysConsts.kSwerveWheelCircumferenceMeters
                    / 60), // UNIT: meters/s
        () ->
            drive_encoder.setMeasurementPeriod(
                Math.max(8, Math.min(period_ms, 64))), // Limit to hall sensor boundaries
        () -> drive_encoder.setPosition(0),
        () -> SparkUtils.setPeriodicFrames(drive_motor, 10, period_ms, period_ms, 0, 0, 0, 0));

    // Configure azimuth motor
    azimuth_motor = new CANSparkMax(constants.azimuth_ID, MotorType.kBrushless);
    SparkUtils.configure(
        azimuth_motor,
        () -> azimuth_motor.setIdleMode(IdleMode.kBrake),
        () -> SparkUtils.setInverted(azimuth_motor, SwerveConsts.kAzimuthInverted),
        () -> azimuth_motor.setSmartCurrentLimit(DriveConsts.kModuleAzimuthCurrentLimit),
        () -> SparkUtils.setPeriodicFrames(azimuth_motor, 10, 0, 0, 0, 0, 0, 0));

    // Configure CANcoder
    cancoder = new CANcoder(constants.cancoder_ID);
    cancoder.getConfigurator().apply(cancoder_config.withMagnetOffset(constants.cancoderOffset));
    var positionSignal = cancoder.getAbsolutePosition();
    positionSignal.setUpdateFrequency(1000d / period_ms);
    rotationSupplier = () -> positionSignal.refresh().getValue() * 360d; // UNIT: ccw degrees

    // Configure azimuth PID controller
    kS = new TunableNumber("S", constants.kS, constants.name);
    kP = new TunableNumber("P", constants.kP, constants.name);
    kD = new TunableNumber("D", constants.kD, constants.name);
    azimuth_controller =
        new PIDController(kP.getAsDouble(), 0, kD.getAsDouble(), period_ms / 1000d);
    kP.bind(azimuth_controller::setP);
    kD.bind(azimuth_controller::setD);
    azimuth_controller.enableContinuousInput(-180, 180);
    azimuth_controller.setSetpoint(0);
  }

  /**
   * Calculate and set swerve module speed.
   *
   * @param speed module speed (UNIT: meters/s)
   * @param angle module angle (UNIT: ccw degrees)
   */
  protected void drive(double speed, double angle) {
    // Calculate and set azimuth motor speed
    azimuth_motor.setVoltage(
        Math.max(
            -DriveConsts.kModuleAzimuthMaxVoltage,
            Math.min(
                DriveConsts.kModuleAzimuthMaxVoltage, // Clamp to nominal voltage
                Math.copySign(kS.getAsDouble(), angle - getAngle()) // Simple static feedforward
                    + azimuth_controller.calculate(getAngle(), angle) // Azimuth feedback controller
                )));

    // Calculate and set drive motor speed
    drive_motor.setVoltage(
        Math.max(
                -PhysConsts.kNEOMaxVoltage,
                Math.min(
                    PhysConsts.kNEOMaxVoltage, // Clamp to nominal voltage
                    Math.copySign(
                            SwerveConsts.kDriveS.getAsDouble(), speed) // Simple static feedforward
                        + DriveConsts.kModuleDriveMaxVoltage
                            / DriveConsts.kMaxLinearVelMetersPerSecond
                            * speed // Simple velocity feedforward
                        + SwerveConsts.kDriveP.getAsDouble()
                            * (speed - getVelocity()) // Feedback controller for velocity adjustment
                    // (helpful for following velocity-reliant pathing)
                    ))
            * Math.abs(
                Math.cos(
                    getAngleError()
                        * Math.PI
                        / 180)) // Scale velocity down if not at proper angle to reduce
        // drag/unintended movement
        );
  }

  /**
   * Drive the swerve module based on a desired angle and speed.
   *
   * @param desired desired state
   */
  protected void desiredStateDrive(SwerveModuleState desired) {
    desired =
        SwerveModuleState.optimize(
            desired,
            Rotation2d.fromDegrees(
                getAngle())); // Optimize so that maximum required turn is 1/4 circle
    drive(desired.speedMetersPerSecond, desired.angle.getDegrees());
  }

  /**
   * @return the angle of the module (UNIT: ccw degrees)
   */
  public double getAngle() {
    return rotationSupplier.get();
  }

  /**
   * @return the velocity of the module (UNIT: meters/s)
   */
  public double getVelocity() {
    return drive_encoder.getVelocity();
  }

  /**
   * @return the current error in the angle of the module (UNIT: ccw degrees)
   */
  public double getAngleError() {
    return azimuth_controller.getPositionError();
  }

  /**
   * @return the distance traveled by the module (UNIT: meters)
   */
  public double getDistance() {
    return drive_encoder.getPosition();
  }

  public void stopMotor() {
    drive_motor.stopMotor();
    azimuth_motor.stopMotor();

    azimuth_controller.reset();
  }

  /** Basic constants for the construction of a {@link SwerveModule}. */
  public static class SwerveModuleConstants {
    public final String name;

    public final double kS;
    public final double kP;
    public final double kD;
    public final double cancoderOffset;

    public final int drive_ID;
    public final int azimuth_ID;
    public final int cancoder_ID;
    public final Translation2d location;

    /**
     * @param name name of the swerve module for data retrieval
     * @param kS azimuth S gain
     * @param kP azimuth P gain
     * @param kD azimuth D gain
     * @param cancoderOffsetRotations additive cancoder offset (UNIT: ccw rotations)
     * @param drive_ID driving motor ID (Spark MAX with NEO V1.1)
     * @param azimuth_ID azimuth motor ID (Spark MAX with NEO V1.1)
     * @param cancoder_ID cancoder ID
     * @param location location of the wheel relative to the center of rotation of the robot
     *     (forward, left) (UNIT: meters)
     */
    public SwerveModuleConstants(
        String name,
        double kS,
        double kP,
        double kD,
        double cancoderOffsetRotations,
        int drive_ID,
        int azimuth_ID,
        int cancoder_ID,
        Translation2d location) {
      this.name = name;
      this.kS = kS;
      this.kP = kP;
      this.kD = kD;
      this.cancoderOffset = cancoderOffsetRotations;
      this.drive_ID = drive_ID;
      this.azimuth_ID = azimuth_ID;
      this.cancoder_ID = cancoder_ID;
      this.location = location;
    }
  }
}
