// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team4201.subsystems.SwerveSubsystem;

public class TrajectoryUtils {
  private final SwerveSubsystem m_swerveDrive;
  private final RobotConfig m_robotConfig;
  private final PIDConstants m_translationPIDConstants;
  private final PIDConstants m_rotationPIDConstants;

  public TrajectoryUtils(SwerveSubsystem swerveDrive, RobotConfig robotConfig, PIDConstants translationPIDConstants, PIDConstants rotationPIDConstants) {
    m_swerveDrive = swerveDrive;
    m_robotConfig = robotConfig;
    m_translationPIDConstants = translationPIDConstants;
    m_rotationPIDConstants = rotationPIDConstants;
  }

  public Command generatePPHolonomicCommand(String pathName, double maxSpeed) {
    try {
     return generatePPHolonomicCommand(PathPlannerPath.fromPathFile(pathName), maxSpeed, false);
    } catch (Exception e) {
      DriverStation.reportError("Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(), e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  public Command generatePPHolonomicCommand(String pathName, double maxSpeed, boolean manualFlip) {
    try {
      return generatePPHolonomicCommand(PathPlannerPath.fromPathFile(pathName), maxSpeed, manualFlip);
    } catch (Exception e) {
        DriverStation.reportError("Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(), e.getStackTrace());
        return new WaitCommand(0);
      }
  }

  public Command generatePPHolonomicCommand(PathPlannerPath path, double maxSpeed) {

    return generatePPHolonomicCommand(path, maxSpeed, false);
  }

  public Command generatePPHolonomicCommand(
      PathPlannerPath path,
      double maxSpeed,
      boolean flipPath) {

    RobotConfig robotConfig = m_robotConfig;
    if (maxSpeed != 0) {
      ModuleConfig moduleConfig =
              new ModuleConfig(m_robotConfig.moduleConfig.wheelRadiusMeters,
                      maxSpeed,
                      m_robotConfig.moduleConfig.wheelCOF,
                      m_robotConfig.moduleConfig.driveMotor,
                      m_robotConfig.moduleConfig.driveCurrentLimit,
                      m_robotConfig.moduleConfig.
                      driveCurrentLimit,
                      1);
      robotConfig = new RobotConfig(m_robotConfig.massKG, m_robotConfig.MOI, moduleConfig, m_robotConfig.moduleLocations);
    }

    return new FollowPathCommand(
        path,
        () -> m_swerveDrive.getState().Pose,
            m_swerveDrive::getChassisSpeeds,
            m_swerveDrive::setChassisSpeedsAuto,
        new PPHolonomicDriveController(
            new PIDConstants(m_translationPIDConstants.kP,
                    m_translationPIDConstants.kI,
                    m_translationPIDConstants.kD),
            new PIDConstants(m_rotationPIDConstants.kP,
                    m_rotationPIDConstants.kI,
                    m_rotationPIDConstants.kD)
                ),
            robotConfig,
        () -> flipPath,
            m_swerveDrive);
  }
}
