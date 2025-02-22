// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.codex.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import org.team4201.codex.subsystems.SwerveSubsystem;

/**
 * Utility class for working with autonomous trajectory following. Is currently designed around <a
 * href="https://github.com/mjansen4857/pathplanner">PathPlanner</a>.
 */
public class TrajectoryUtils {
  private final SwerveSubsystem m_swerveDrive;

  /**
   * Construct a {@link TrajectoryUtils} object to manage trajectory following.
   *
   * @param swerveDrive The {@link SwerveSubsystem}
   */
  public TrajectoryUtils(SwerveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param pathName The name of the PathPlanner Trajectory file to reference.
   * @return Command
   */
  public Command generatePPHolonomicCommand(String pathName) {
    try {
      return generatePPHolonomicCommand(
          PathPlannerPath.fromPathFile(pathName), this::flipPathByAlliance);
    } catch (Exception e) {
      DriverStation.reportError(
          "Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(),
          e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param pathName The name of the PathPlanner Trajectory file to reference.
   * @param flipPath Option to flip the trajectory instead of using the robot's current {@link
   *     DriverStation.Alliance} color
   * @return Command
   */
  public Command generatePPHolonomicCommand(String pathName, BooleanSupplier flipPath) {
    try {
      return generatePPHolonomicCommand(PathPlannerPath.fromPathFile(pathName), flipPath);
    } catch (Exception e) {
      DriverStation.reportError(
          "Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(),
          e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param path The {@link PathPlannerPath} to follow.
   * @return Command
   */
  public Command generatePPHolonomicCommand(PathPlannerPath path) {

    return generatePPHolonomicCommand(path, this::flipPathByAlliance);
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param path The {@link PathPlannerPath} to follow.
   * @param flipPath Option to flip the trajectory around the field's center.
   * @return Command
   */
  public Command generatePPHolonomicCommand(PathPlannerPath path, BooleanSupplier flipPath) {
    return new FollowPathCommand(
        path,
        () -> m_swerveDrive.getState().Pose,
        () -> m_swerveDrive.getState().Speeds,
        m_swerveDrive::setChassisSpeedsAuto,
        new PPHolonomicDriveController(
            m_swerveDrive.getAutoTranslationPIDConstants(),
            m_swerveDrive.getAutoRotationPIDConstants()),
        m_swerveDrive.getAutoRobotConfig(),
        flipPath,
        m_swerveDrive);
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param pathName The name of the {@link PathPlannerPath} containing the initial {@link Pose2d}.
   * @return Command
   */
  public Command resetRobotPoseAuto(String pathName) {
    try {
      return resetRobotPoseAuto(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      DriverStation.reportError(
          "Could not load PathPlanner Path '" + pathName + "':" + e.getMessage(),
          e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param path The {@link PathPlannerPath} containing the initial {@link Pose2d}.
   * @return Command
   */
  public Command resetRobotPoseAuto(PathPlannerPath path) {
    return new InstantCommand(
        () -> {
          var pathStartingPose = path.getStartingHolonomicPose();
          AtomicReference<Pose2d> startingPose = new AtomicReference<>(Pose2d.kZero);

          pathStartingPose.ifPresent(
              (p) -> {
                if (flipPathByAlliance()) {
                  startingPose.set(FlippingUtil.flipFieldPose(p));
                } else {
                  startingPose.set(p);
                }
              });

          m_swerveDrive.resetPose(startingPose.get());
        },
        m_swerveDrive);
  }

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}. NOTE: This
   * constructor should only be used for generating Trajectories used for displays.
   *
   * @param paths the {@link PathPlannerPath}(s) to use
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(PathPlannerPath... paths) {
    return getTrajectoryFromPathPlanner(new TrajectoryConfig(1, 1), paths);
  }

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}
   *
   * @param config The WPILib {@link TrajectoryConfig}.
   * @param paths the {@link PathPlannerPath}(s) to use
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(
      TrajectoryConfig config, PathPlannerPath... paths) {
    return getTrajectoryFromPathPlanner(config, false, paths);
  }

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}
   *
   * @param config The WPILib {@link TrajectoryConfig}.
   * @param flipPath Whether to flip the {@link PathPlannerPath} before displaying it.
   * @param paths the {@link PathPlannerPath}(s) to use
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(
      TrajectoryConfig config, boolean flipPath, PathPlannerPath... paths) {
    var pathPoses = new ArrayList<Pose2d>();

    for (var path : paths) {
      if (flipPath) {
        pathPoses.addAll(
            path.flipPath().getPathPoses().stream()
                .map(
                    p ->
                        new Pose2d(
                            p.getTranslation(), p.getRotation().rotateBy(Rotation2d.k180deg)))
                .toList());
      } else {
        pathPoses.addAll(path.getPathPoses());
      }
    }

    return TrajectoryGenerator.generateTrajectory(pathPoses, config);
  }

  private boolean flipPathByAlliance() {
    return DriverStation.getAlliance()
        .filter(alliance -> alliance == DriverStation.Alliance.Red)
        .isPresent();
  }
}
