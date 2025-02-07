package org.team4201.codex.utils;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team4201.codex.subsystems.TestSwerveSubsystem;

public class TestTrajectoryUtils {
  private TrajectoryUtils m_trajectoryUtils;

  @BeforeEach
  public void constructDevices() {
    SwerveDrivetrainConstants driveConstants = new SwerveDrivetrainConstants();

    SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[4];
    Translation2d[] modulePositions = {
      new Translation2d(0.5, 0.5),
      new Translation2d(0.5, -0.5),
      new Translation2d(-0.5, 0.5),
      new Translation2d(-0.5, -0.5)
    };
    SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(1)
                .withSteerMotorGearRatio(1);

    for (int i = 0; i < 4; i++) {
      moduleConstants[i] =
          ConstantCreator.createModuleConstants(
              i,
              i + 4,
              i,
              0,
              modulePositions[i].getX(),
              modulePositions[i].getY(),
              false,
              false,
              false);
    }

    TestSwerveSubsystem.SwerveDrive swerveDrive =
        new TestSwerveSubsystem.SwerveDrive(driveConstants, moduleConstants);

    m_trajectoryUtils =
        new TrajectoryUtils(
            swerveDrive,
            new RobotConfig(
                0,
                0,
                new ModuleConfig(0, 0, 0, DCMotor.getKrakenX60(1), 0, 0),
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5)),
            new PIDConstants(0),
            new PIDConstants(0));
  }

  @Test
  public void testTrajectoryConversion() {
    var pathPoints = new ArrayList<PathPoint>();
    pathPoints.add(new PathPoint(new Translation2d(0, 0)));
    pathPoints.add(new PathPoint(new Translation2d(10, 0)));

    var path =
        PathPlannerPath.fromPathPoints(
            pathPoints, new PathConstraints(1, 1, 1, 1, 12), new GoalEndState(0, Rotation2d.kZero));

    // Just check that the function runs
    assertInstanceOf(Trajectory.class, m_trajectoryUtils.getTrajectoryFromPathPlanner(path));
  }

  @Test
  public void testTrajectoryRotationalFlip() {
    var pathConstraints = new PathConstraints(1, 1, 1, 1, 12);
    var trajectoryConfig = new TrajectoryConfig(1, 1);
    var pathPoints = new ArrayList<PathPoint>();
    pathPoints.add(
        new PathPoint(
            new Translation2d(0, 0),
            new RotationTarget(0, Rotation2d.fromDegrees(45)),
            pathConstraints));
    pathPoints.add(
        new PathPoint(
            new Translation2d(4, 4),
            new RotationTarget(0, Rotation2d.fromDegrees(45)),
            pathConstraints));

    var path =
        PathPlannerPath.fromPathPoints(
            pathPoints, pathConstraints, new GoalEndState(0, Rotation2d.fromDegrees(45)));

    var blueTrajectory =
        m_trajectoryUtils.getTrajectoryFromPathPlanner(trajectoryConfig, false, path);
    var blueEndState =
        new Trajectory.State(
            blueTrajectory.getTotalTimeSeconds(),
            0,
            0,
            new Pose2d(4, 4, Rotation2d.fromDegrees(45)),
            0);
    // Acceleration of WPILib Trajectory ends with - 1/s^2?
    // assertEquals(blueEndState, blueTrajectory.sample(blueTrajectory.getTotalTimeSeconds()));
    assertEquals(
        blueEndState.poseMeters,
        blueTrajectory.sample(blueTrajectory.getTotalTimeSeconds()).poseMeters);

    var redTrajectory =
        m_trajectoryUtils.getTrajectoryFromPathPlanner(trajectoryConfig, true, path);
    var redEndState =
        new Trajectory.State(
            redTrajectory.getTotalTimeSeconds(),
            0,
            0,
            new Pose2d(
                FlippingUtil.fieldSizeX - 4,
                FlippingUtil.fieldSizeY - 4,
                Rotation2d.fromDegrees(45).minus(Rotation2d.k180deg)),
            0);
    // Acceleration of WPILib Trajectory ends with - 1/s^2?
    // assertEquals(redEndState, redTrajectory.sample(redTrajectory.getTotalTimeSeconds()));
    assertEquals(
        redEndState.poseMeters,
        redTrajectory.sample(redTrajectory.getTotalTimeSeconds()).poseMeters);
  }
}
