// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.ExtrapolatingDoubleTreeMap;

/** Add your docs here. */
public class ShotCalculator {
  private static final Translation2d redHubPose = new Translation2d(11.915394, 4.021328);
  public static final Translation2d blueHubPose = new Translation2d(4.625594, 4.021328);
  private static final Translation2d blueRightPass = new Translation2d(1.15, 1.177);
  private static Translation2d targetPose = Translation2d.kZero;
  private static final int NumItterations = 5;

  public record ShootingSolution(Angle turretAngle, Angle hoodAngle, double flywheelSpeed) {}

  private static ExtrapolatingDoubleTreeMap tofMap = new ExtrapolatingDoubleTreeMap();

  static {
    tofMap.put(5.759, 5.875 * .25);
    tofMap.put(4.561, 6.225 * .25);
    tofMap.put(3.786, 5.335 * .25);
    tofMap.put(2.93, 5.0 * .25);
    tofMap.put(2.578, 5.063 * .25);
  }

  private static InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    hoodMap.put(5.759, 33.0);
    hoodMap.put(4.561, 30.0);
    hoodMap.put(3.786, 30.0);
    hoodMap.put(2.93, 28.0);
    hoodMap.put(2.578, 26.0);
    hoodMap.put(1.7, 23.0);
  }

  private static InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  static {
    flywheelMap.put(5.759, 94.0);
    flywheelMap.put(4.561, 90.0);
    flywheelMap.put(3.786, 81.0);
    flywheelMap.put(2.93, 75.0);
    flywheelMap.put(2.578, 76.0);
    flywheelMap.put(1.7, 75.0);
  }

  public static double getFlywheelSpeed(double distance) {
    return flywheelMap.get(distance);
  }

  public static double getHoodPosition(double distance) {
    return hoodMap.get(distance);
  }

  public static double getTOF(double distance) {
    return tofMap.get(distance);
  }

  public static ShootingSolution getStaticHubSolution(Pose2d robotPose) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      targetPose = blueHubPose;
    } else {
      targetPose = redHubPose;
    }

    Translation2d difference = targetPose.minus(robotPose.getTranslation());
    double distance = difference.getNorm();

    Angle turretAngle = difference.getAngle().minus(robotPose.getRotation()).getMeasure();

    return new ShootingSolution(turretAngle, Degrees.of(hoodMap.get(distance)), flywheelMap.get(distance));
  }

  public static ShootingSolution getPassingSolution(Pose2d robotPose) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      targetPose = blueRightPass;
    } else {
      targetPose = redHubPose;
    }

    Translation2d difference = targetPose.minus(robotPose.getTranslation());
    double distance = difference.getNorm();

    Angle turretAngle = difference.getAngle().minus(robotPose.getRotation()).getMeasure();

    return new ShootingSolution(turretAngle, Degrees.of(hoodMap.get(distance)), flywheelMap.get(distance) - 10.0);
  }

  public static ShootingSolution getSOTMhubSolution(Pose2d robotPose, Translation2d robotVelocity) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      targetPose = blueHubPose;
    } else {
      targetPose = redHubPose;
    }

    Translation2d difference = targetPose.minus(robotPose.getTranslation());
    double distance = difference.getNorm();

    // Itterate shot projection to hopefully converge on correct shot
    for (int i = 0; i < NumItterations; i++) {
      distance = robotPose.getTranslation().getDistance(targetPose);
      double tof = tofMap.get(distance);
      targetPose = targetPose.minus(
          robotVelocity.times(tof)); // Shift goal by predicted change in flight due to robot velocity
    }

    difference = targetPose.minus(robotPose.getTranslation());
    Angle turretAngle = difference.getAngle().minus(robotPose.getRotation()).getMeasure();

    // TODO: Check if shot is good (there are situations where it diverges or converges too slowly)

    return new ShootingSolution(turretAngle, Degrees.of(hoodMap.get(distance)), flywheelMap.get(distance));
  }
}
