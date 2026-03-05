// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher.ShotCalculator.ShootingSolution;

@Logged
public class Launcher extends SubsystemBase {

  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Turret turret = new Turret();
  private CommandSwerveDrivetrain drivetrain;

  private ShootingSolution bestShootingSolution;

  private static Translation2d turretOffset = new Translation2d(Inches.of(-6), Inches.of(-12));

  /** Creates a new Launcher. */
  public Launcher(CommandSwerveDrivetrain Drivetrain) {
    drivetrain = Drivetrain;

    SmartDashboard.putData("Flywheel", flywheel);
    SmartDashboard.putData("Hood", hood);
    SmartDashboard.putData("Turret", turret);
    SmartDashboard.putData("Launcher", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTurretPose();
  }

  private Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }

  public Command runToZero() {
    return expose(flywheel.idle()
            .alongWith(hood.targetAngle(() -> Rotation.of(0)))
            .alongWith(turret.targetAngle(() -> Rotation.of(0))))
        .withName("Run to zero");
  }

  public Command targetHub() {
    return expose(run(() -> {
              var curentPose = drivetrain.getEstimatedPose();
              var turretPose = curentPose.transformBy(
                  new Transform2d(turretOffset.rotateBy(curentPose.getRotation()), Rotation2d.kZero));
              bestShootingSolution = ShotCalculator.getStaticHubSolution(turretPose);
            })
            .alongWith(targetBest()))
        .withName("TargetHub");
  }

  private Command targetBest() {
    return flywheel.runAtVelocity(() -> bestShootingSolution.flywheelSpeed())
        .alongWith(hood.targetAngle(() -> bestShootingSolution.hoodAngle()))
        .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()));
  }

  private Pose2d getTurretPose() {
    var curentPose = drivetrain.getEstimatedPose();
    return curentPose.transformBy(
        new Transform2d(turretOffset.rotateBy(curentPose.getRotation()), Rotation2d.kZero));
  }
}
