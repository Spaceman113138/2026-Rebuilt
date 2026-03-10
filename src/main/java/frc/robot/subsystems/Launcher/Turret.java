// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

@Logged
class Turret extends SubsystemBase {

  private TalonFX turretMotor = new TalonFX(Constants.turretId, TunerConstants.kCANBus);
  private CANcoder smallEncoder = new CANcoder(Constants.smallTurretEncoderId, TunerConstants.kCANBus);
  private CANcoder largeEncoder = new CANcoder(Constants.largeTurretEncoderId, TunerConstants.kCANBus);

  private PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private static final Angle minRotation = Rotations.of(-0.5);
  private static final Angle maxRotation = Rotations.of(0.5);
  private static final double gearRatio = 3.0 * (100.0 / 10.0);
  private static final double largeEncoderTeeth = 14.0;
  private static final double smallEncoderTeeth = 13.0;

  private EasyCRT crt;

  private NetworkTableEntry turretEntry = NetworkTableInstance.getDefault().getEntry("/tuning/turretTarget");

  /** Creates a new Turret. */
  public Turret() {
    turretEntry.getTopic().genericPublish("double");
    turretEntry.getTopic().setPersistent(true);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(120)
        .withSupplyCurrentLimit(40);
    config.ClosedLoopGeneral.withContinuousWrap(false);
    config.Feedback.withSensorToMechanismRatio(gearRatio);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxRotation)
        .withReverseSoftLimitThreshold(minRotation);
    config.Slot0.withKP(70.0).withKD(0.0).withKS(0.43).withKV(3.42);

    turretMotor.getConfigurator().apply(config);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    smallEncoder.getConfigurator().apply(encoderConfig);
    largeEncoder.getConfigurator().apply(encoderConfig);

    EasyCRTConfig crtConfig = new EasyCRTConfig(
        smallEncoder.getAbsolutePosition(true).asSupplier(),
        largeEncoder.getAbsolutePosition(true).asSupplier());
    crtConfig
        .withEncoderRatios(100.0 / smallEncoderTeeth, 100.0 / largeEncoderTeeth)
        .withMechanismRange(minRotation, maxRotation)
        .withMatchTolerance(Rotations.of(0.06));

    crt = new EasyCRT(crtConfig);
    turretMotor.setPosition(0);
    SmartDashboard.putData("TargetTurret", targetDashboardAngle());
    // crt.getAngleOptional().ifPresentOrElse((angle) -> turretMotor.setPosition(angle), () ->
    // turretMotor.setPosition(Rotations.of(0.0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Angle getRotation() {
    return turretMotor.getPosition().getValue();
  }

  protected Command targetAngle(Supplier<Angle> targetAngle) {
    return run(() -> turretMotor.setControl(
        positionRequest.withPosition(targetAngle.get()).withVelocity(0)));
  }

  protected Command targetAngleWithVelocity(Supplier<Angle> targetAngle, Supplier<AngularVelocity> targetVelocity) {
    return run(() -> turretMotor.setControl(
        positionRequest.withPosition(targetAngle.get()).withVelocity(targetVelocity.get())));
  }

  public boolean atTarget() {
    return positionRequest
        .getPositionMeasure()
        .isNear(turretMotor.getPosition().getValue(), Degrees.of(1.0));
  }

  // Assume target angle is within a single rotation [-0.5 , 0.5]
  private Angle wrapTargetAngle(Angle target, boolean wrapAggresive) {

    Angle minTargetAngle = minRotation;
    Angle maxTargetAngle = maxRotation;

    if (wrapAggresive) {
      minTargetAngle = Degrees.of(-190);
      maxTargetAngle = Degrees.of(190);
    }
    return Degrees.of(0);
  }

  private Command targetDashboardAngle() {
    return targetAngle(() -> Degrees.of(turretEntry.getDouble(0)));
  }

  // Assume target angle is within a single rotation
  // private Angle wrapTargetAngle(Angle target, boolean wrapAggresive) {
  //   if (!wrapAggresive) {
  //     Angle potentialTarget = target.minus(Rotations.of(1));
  //     if (target.lt(Rotations.of(0))) {
  //       potentialTarget = target.plus(Rotations.of(1));
  //     }

  //     if ()
  //   }
  // }
}
