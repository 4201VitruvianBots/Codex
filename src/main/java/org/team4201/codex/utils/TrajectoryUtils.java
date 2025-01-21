// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.codex.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team4201.codex.subsystems.SwerveSubsystem;

import java.util.ArrayList;

/**
 * Utility class for working with autonomous trajectory following. Is currently designed around
 * <a href="https://github.com/mjansen4857/pathplanner">PathPlanner</a>.
 */
public class TrajectoryUtils {
  private final SwerveSubsystem m_swerveDrive;
  private final RobotConfig m_robotConfig;
  private final PIDConstants m_translationPIDConstants;
  private final PIDConstants m_rotationPIDConstants;

  /**
   * Construct a {@link TrajectoryUtils} object to manage trajectory following.
   *
   * @param swerveDrive The {@link SwerveSubsystem}
   * @param robotConfig The PathPlanner {@link RobotConfig}
   * @param translationPIDConstants The PathPlanner translation {@link PIDConstants}
   * @param rotationPIDConstants The PathPlanner rotation {@link PIDConstants}
   */
  public TrajectoryUtils(SwerveSubsystem swerveDrive, RobotConfig robotConfig, PIDConstants translationPIDConstants, PIDConstants rotationPIDConstants) {
    m_swerveDrive = swerveDrive;
    m_robotConfig = robotConfig;
    m_translationPIDConstants = translationPIDConstants;
    m_rotationPIDConstants = rotationPIDConstants;
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param pathName The name of the PathPlanner Trajectory file to reference.
   *
   * @return Command
   */
  public Command generatePPHolonomicCommand(String pathName) {
    try {
     return generatePPHolonomicCommand(PathPlannerPath.fromPathFile(pathName), m_robotConfig, false);
    } catch (Exception e) {
      DriverStation.reportError("Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(), e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param pathName The name of the PathPlanner Trajectory file to reference.
   * @param manualFlip Option to manually flip the trajectory instead of using the robot's current {@link DriverStation.Alliance} color
   *
   * @return Command
   */
  public Command generatePPHolonomicCommand(String pathName, boolean manualFlip) {
    try {
      return generatePPHolonomicCommand(PathPlannerPath.fromPathFile(pathName), m_robotConfig, manualFlip);
    } catch (Exception e) {
        DriverStation.reportError("Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(), e.getStackTrace());
        return new WaitCommand(0);
      }
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param path The {@link PathPlannerPath} to follow.
   * @param robotConfig The {@link RobotConfig} to use.
   *
   * @return Command
   */
  public Command generatePPHolonomicCommand(PathPlannerPath path, RobotConfig robotConfig) {

    return generatePPHolonomicCommand(path, robotConfig, false);
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param path The {@link PathPlannerPath} to follow.
   * @param robotConfig The {@link RobotConfig} to use
   * @param flipPath Option to flip the trajectory around the field's center.
   *
   * @return Command
   */
  public Command generatePPHolonomicCommand(
      PathPlannerPath path,
      RobotConfig robotConfig,
      boolean flipPath) {
    return new FollowPathCommand(
        path,
        () -> m_swerveDrive.getState().Pose,
            ()-> m_swerveDrive.getState().Speeds,
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

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}
   *
   * @param paths the {@link PathPlannerPath}(s) to use
   *
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(PathPlannerPath... paths) {
    return getTrajectoryFromPathPlanner(false, paths);
  }
  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}
   *
   * @param flipPath Whether to flip the {@link PathPlannerPath} before displaying it.
   * @param paths the {@link PathPlannerPath}(s) to use
   *
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(boolean flipPath, PathPlannerPath... paths) {
    var pathPoses = new ArrayList<Pose2d>();

    for (var path : paths) {
      if (flipPath) {
        path = path.flipPath();
      }
      pathPoses.addAll(path.getPathPoses());
    }
    return TrajectoryGenerator.generateTrajectory(pathPoses, new TrajectoryConfig(1, 1));
  }
}
