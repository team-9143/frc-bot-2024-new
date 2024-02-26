package frc.robot.util;

import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.SwerveConsts;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Controls a single swerve module. */
public class SwerveModule {
  private static final MagnetSensorConfigs cancoder_config = new MagnetSensorConfigs()
    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

  private final CANSparkMax drive_motor;
  private final CANSparkMax azimuth_motor;

  private final RelativeEncoder drive_encoder;
  private final CANcoder cancoder;
  private final Supplier<Double> rotationSupplier;

  private final PIDController speed_controller;
  private final PIDController azimuth_controller;

  protected SwerveModule(SwerveModuleConstants constants) {
    drive_motor = new CANSparkMax(constants.drive_ID, MotorType.kBrushless);
    azimuth_motor = new CANSparkMax(constants.azimuth_ID, MotorType.kBrushless);
    azimuth_motor.setInverted(SwerveConsts.kAzimuthInverted);

    // Configure drive encoder
    drive_encoder = drive_motor.getEncoder();
    drive_encoder.setPositionConversionFactor(PhysConsts.kSwerveDriveGearbox * PhysConsts.kSwerveWheelCircumferenceMeters); // UNIT: meters
    drive_encoder.setVelocityConversionFactor(PhysConsts.kSwerveDriveGearbox * PhysConsts.kSwerveWheelCircumferenceMeters / 60); // UNIT: meters/s
    drive_encoder.setMeasurementPeriod(20);
    drive_encoder.setPosition(0);

    // Configure CANcoder
    cancoder = new CANcoder(constants.cancoder_ID);
    cancoder.getConfigurator().apply(cancoder_config.withMagnetOffset(constants.cancoderOffset));
    var positionSignal = cancoder.getAbsolutePosition();
    positionSignal.setUpdateFrequency(50);
    rotationSupplier = () -> positionSignal.refresh().getValue() * 360d; // UNIT: ccw degrees

    // Configure speed PID controller
    speed_controller = new PIDController(SwerveConsts.kDriveP.getAsDouble(), 0, 0);
    SwerveConsts.kDriveP.bind(speed_controller::setP);
    speed_controller.setSetpoint(0);

    // Configure azimuth PID controller
    azimuth_controller = new PIDController(SwerveConsts.kAzimuthP.getAsDouble(), 0, SwerveConsts.kAzimuthD.getAsDouble());
    SwerveConsts.kAzimuthP.bind(azimuth_controller::setP);
    SwerveConsts.kAzimuthD.bind(azimuth_controller::setD);
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
      Math.max(-DriveConsts.kMaxModuleAzimuthVoltage, Math.min(DriveConsts.kMaxModuleAzimuthVoltage, // Clamp to nominal voltage
        SwerveConsts.kAzimuthS.getAsDouble() * Math.signum(angle - getAngle()) // Simple static feedforward
        + azimuth_controller.calculate(getAngle(), angle) // Azimuth feedback controller
      ))
    );

    // Calculate and set drive motor speed
    drive_motor.setVoltage(
      Math.max(-PhysConsts.kNEOMaxVoltage, Math.min(PhysConsts.kNEOMaxVoltage, // Clamp to nominal voltage
        SwerveConsts.kDriveS.getAsDouble() * Math.signum(speed) // Simple static feedforward
        + (PhysConsts.kNEOMaxVoltage * speed/DriveConsts.kMaxLinearVelMetersPerSecond) // Simple velocity feedforward
        + speed_controller.calculate(getVelocity(), speed) // Velocity adjustment feedback controller
      )) * Math.abs(Math.cos(getAngleError() * Math.PI/180)) // Scale velocity down if not at proper angle
    );
  }

  /**
   * Drive the swerve module based on a desired angle and speed.
   *
   * @param desired desired state
   */
  protected void desiredStateDrive(SwerveModuleState desired) {
    desired = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle())); // Optimize so that maximum required turn is 1/4 circle
    drive(
      desired.speedMetersPerSecond,
      desired.angle.getDegrees()
    );
  }

  /** @return the angle of the module (UNIT: ccw degrees) */
  public double getAngle() {
    return rotationSupplier.get();
  }

  /** @return the velocity of the module (UNIT: meters/s) */
  public double getVelocity() {
    return drive_encoder.getVelocity();
  }

  /** @return the current error in the angle of the module (UNIT: ccw degrees) */
  public double getAngleError() {
    return azimuth_controller.getPositionError();
  }

  /** @return the distance traveled by the module (UNIT: meters) */
  public double getDistance() {
    return drive_encoder.getPosition();
  }

  public void stopMotor() {
    drive_motor.stopMotor();
    azimuth_motor.stopMotor();

    azimuth_controller.reset();
    speed_controller.reset();
  }

  /** Basic constants for the construction of a {@link SwerveModule}. */
  public static class SwerveModuleConstants {
    public final int drive_ID;
    public final int azimuth_ID;
    public final int cancoder_ID;

    public final double cancoderOffset;
    public final Translation2d location;

    /**
     * @param drive_ID driving motor ID (Spark Max with brushless motor)
     * @param azimuth_ID azimuth motor ID (Spark Max with brushless motor)
     * @param cancoder_ID cancoder ID
     * @param cancoderOffsetRotations additive cancoder offset (UNIT: ccw rotations)
     * @param location location of the wheel relative to the center of rotation of the robot (forward, left) (UNIT: meters)
     */
    public SwerveModuleConstants(int drive_ID, int azimuth_ID, int cancoder_ID, double cancoderOffsetRotations, Translation2d location) {
      this.drive_ID = drive_ID;
      this.azimuth_ID = azimuth_ID;
      this.cancoder_ID = cancoder_ID;
      this.cancoderOffset = cancoderOffsetRotations;
      this.location = location;
    }
  }
}