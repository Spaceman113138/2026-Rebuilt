// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher.ShotCalculator.ShootingSolution;

@Logged
public class Launcher extends SubsystemBase {

  public final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Turret turret = new Turret();
  private CommandSwerveDrivetrain drivetrain;

  private ShootingSolution bestShootingSolution = new ShootingSolution(Degrees.of(0), Degrees.of(0), 0);

  private static Translation2d turretOffset = new Translation2d(Inches.of(-2.684942), Inches.of(-3.674131));

  public Trigger launcherReady = new Trigger(() -> flywheel.atTarget() && hood.atTarget() && turret.atTarget());

  private boolean currentlyShooting = false;
  private Pose2d mostRecentTarget = new Pose2d();
  private Pose2d[] intermediateTargets = new Pose2d[20];
  private double distance = 0.0;

  // private LinearFilter distanceFilter = LinearFilter.movingAverage(20);

  /** Creates a new Launcher. */
  public Launcher(CommandSwerveDrivetrain Drivetrain) {
    drivetrain = Drivetrain;

    // SmartDashboard.putData("Flywheel", flywheel);
    // SmartDashboard.putData("Hood", hood);
    // SmartDashboard.putData("Turret", turret);
    // SmartDashboard.putData("Launcher", this);

    setDefaultCommand(runToZero());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = getTurretPose().getTranslation().getDistance(ShotCalculator.blueHubPose);

    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      if (getTurretPose().getX() < 4.625594) {
        bestShootingSolution = currentlyShooting
            ? ShotCalculator.getSOTMhubSolution(getTurretPose(), drivetrain.getFieldReletiveVelocity())
            : ShotCalculator.getStaticHubSolution(getTurretPose());
      } else {
        bestShootingSolution = ShotCalculator.getPassingSolution(getTurretPose());
      }
    } else {
      if (getTurretPose().getX() > 16.540988 - 4.625594) {
        bestShootingSolution = currentlyShooting
            ? ShotCalculator.getSOTMhubSolution(getTurretPose(), drivetrain.getFieldReletiveVelocity())
            : ShotCalculator.getStaticHubSolution(getTurretPose());
      } else {
        bestShootingSolution = ShotCalculator.getPassingSolution(getTurretPose());
      }
    }

    intermediateTargets = ShotCalculator.intermediateTargets;
    mostRecentTarget = ShotCalculator.mostRecentTarget;
  }

  private Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }

  public Command runToZero() {
    return expose(flywheel.idleCommand()
            .alongWith(hood.targetAngle(() -> Rotation.of(0)))
            .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle())))
        .withName("Run to zero");
  }

  public Command targetHub() {
    return expose(targetBest()).withName("TargetHub");
  }

  private Command targetBest() {
    return flywheel.runAtVelocity(() -> bestShootingSolution.flywheelSpeed())
        .alongWith(hood.targetAngle(() -> bestShootingSolution.hoodAngle()))
        .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()))
        .alongWith(Commands.startEnd(() -> currentlyShooting = true, () -> currentlyShooting = false));
  }

  public Command targetDashboard() {
    return expose(flywheel.runAtDashboardVelocity()
        .alongWith(hood.targetDashboardAngle()
            .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()))));
  }

  public Command testCommand() {
    return expose(flywheel.runAtVelocity(() -> 20.0)
        .alongWith(hood.targetAngle(() -> bestShootingSolution.hoodAngle()))
        .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()))
        .alongWith(Commands.startEnd(() -> currentlyShooting = true, () -> currentlyShooting = false)));
  }

  public Pose2d getTurretPose() {
    var curentPose = drivetrain.getEstimatedPose();
    return new Pose2d(
        curentPose.getTranslation().plus(turretOffset.rotateBy(curentPose.getRotation())),
        curentPose.getRotation());
  }
}
