// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
class Flywheel extends SubsystemBase {

  private TalonFX leftFlywheelMotor = new TalonFX(Constants.leftFlyweelId, CANBus.roboRIO());
  private TalonFX rightFlywheelMotor = new TalonFX(Constants.rightFlywheelId, CANBus.roboRIO());

  private static final double motorToFlywheelRatio = 15.0/18.0;

  private VelocityTorqueCurrentFOC velocityControlRequest = new VelocityTorqueCurrentFOC(motorToFlywheelRatio).withSlot(0);
  private NeutralOut neutralRequest = new NeutralOut(); 


  /** Creates a new Flywheel. */
  public Flywheel() {

    leftFlywheelMotor.getConfigurator().apply(motorConfig());
    rightFlywheelMotor.getConfigurator().apply(motorConfig());
    rightFlywheelMotor.setControl(new Follower(Constants.leftFlyweelId, MotorAlignmentValue.Opposed));

  }

  private TalonFXConfiguration motorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(120)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60);
    
    motorConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.Clockwise_Positive);

    motorConfig.Feedback
      .withSensorToMechanismRatio(motorToFlywheelRatio);

    motorConfig.Slot0
      .withKP(0.0)
      .withKD(0.0)
      .withKS(0.0)
      .withKV(0.0);



    return motorConfig;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  protected Command idleCommand() {
    return startRun(
      () -> leftFlywheelMotor.setControl(neutralRequest), ()->{});
  }

  protected Command runAtVelocity(DoubleSupplier desiredVelocity) {
    return startRun(() -> leftFlywheelMotor.setControl(velocityControlRequest.withVelocity(desiredVelocity.getAsDouble())), ()->{});
  }

}
