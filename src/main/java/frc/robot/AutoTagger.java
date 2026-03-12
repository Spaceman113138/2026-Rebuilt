// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class AutoTagger {
  private PathPlannerPath depotPath;
  private boolean depotAvalible = true;
  private Alert depotAlert = new Alert("Depot Path NOT Found", AlertType.kWarning);

  private SendableChooser<Command> tagChooser = new SendableChooser<>();

  PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public AutoTagger(CommandSwerveDrivetrain drivetrain) {
    var idleRequest = new SwerveRequest.Idle();
    tagChooser.setDefaultOption("None", drivetrain.applyRequest(() -> idleRequest));

    try {
      depotPath = PathPlannerPath.fromPathFile("depot");
      tagChooser.addOption("Depot", getDepot());
    } catch (Exception e) {
      depotAvalible = false;
      depotAlert.set(true);
    }
  }

  public SendableChooser<Command> getChosser() {
    return tagChooser;
  }

  private Command getDepot() {
    return AutoBuilder.pathfindThenFollowPath(depotPath, constraints);
  }
}
