package org.team4201.codex.utils;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team4201.codex.subsystems.MockSwerveSubsystem;

public class TestTrajectoryUtils {
  private TrajectoryUtils m_trajectoryUtils;

  @BeforeEach
  public void constructDevices() {
    MockSwerveSubsystem swerveDrive = new MockSwerveSubsystem();

    m_trajectoryUtils = new TrajectoryUtils(swerveDrive);
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
