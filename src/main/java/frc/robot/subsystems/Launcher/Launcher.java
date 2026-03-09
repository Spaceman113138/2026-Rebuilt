// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher.ShotCalculator.ShootingSolution;

@Logged
public class Launcher extends SubsystemBase {

  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Turret turret = new Turret();
  private CommandSwerveDrivetrain drivetrain;

  private ShootingSolution bestShootingSolution = new ShootingSolution(Degrees.of(0), Degrees.of(0), 0);

  private static Translation2d turretOffset = new Translation2d(Inches.of(-2.684942), Inches.of(-3.674131));

  public Trigger launcherReady = new Trigger(() -> flywheel.atTarget() && hood.atTarget() && turret.atTarget());

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
              bestShootingSolution = ShotCalculator.getStaticHubSolution(getTurretPose());
            })
            .alongWith(targetBest()))
        .withName("TargetHub");
  }

  private Command targetBest() {
    return flywheel.runAtVelocity(() -> bestShootingSolution.flywheelSpeed())
        .alongWith(hood.targetAngle(() -> bestShootingSolution.hoodAngle()))
        .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()));
  }

  public Command targetDashboard() {
    return expose(flywheel.runAtDashboardVelocity().alongWith(hood.targetDashboardAngle()));
  }

  public Pose2d getTurretPose() {
    var curentPose = drivetrain.getEstimatedPose();
    return curentPose.transformBy(
        new Transform2d(turretOffset.rotateBy(curentPose.getRotation()), new Rotation2d(turret.getRotation())));
  }
}
