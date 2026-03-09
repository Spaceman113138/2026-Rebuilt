// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LoggingFiles;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.subsystems.Launcher.ShotCalculator.ShootingSolution;

/** Add your docs here. */
@CustomLoggerFor(ShootingSolution.class)
public class ShootingSolutionLogger extends ClassSpecificLogger<ShootingSolution> {
  public ShootingSolutionLogger() {
    super(ShootingSolution.class);
  }

  @Override
  protected void update(EpilogueBackend backend, ShootingSolution solution) {
    backend.log("flywheel target", solution.flywheelSpeed());
    backend.log("hood target", solution.hoodAngle());
    backend.log("turret target", solution.turretAngle());
  }
}
