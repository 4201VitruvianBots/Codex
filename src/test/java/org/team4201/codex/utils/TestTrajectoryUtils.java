package org.team4201.codex.utils;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.team4201.codex.subsystems.MockSwerveSubsystem;

public class TestTrajectoryUtils {
  private MockSwerveSubsystem m_swerveDrive;
  private TrajectoryUtils m_trajectoryUtils;

  @BeforeEach
  public void constructDevices() {
    m_trajectoryUtils = new TrajectoryUtils(m_swerveDrive);

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().enable();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    CommandScheduler.getInstance().clearComposedCommands();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);

    m_swerveDrive = new MockSwerveSubsystem();
    m_trajectoryUtils = new TrajectoryUtils(m_swerveDrive);
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
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(0, 0, Rotation2d.kZero), new Pose2d(4, 0, Rotation2d.kZero));
    var path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            new IdealStartingState(0, Rotation2d.kZero),
            new GoalEndState(0, Rotation2d.kZero));

    var blueTrajectory = m_trajectoryUtils.getTrajectoryFromPathPlanner(() -> false, path);

    var blueEndPose = new Pose2d(waypoints.get(1).anchor(), Rotation2d.kZero);

    assertEquals(
        blueEndPose, blueTrajectory.sample(blueTrajectory.getTotalTimeSeconds()).poseMeters);

    var redTrajectory = m_trajectoryUtils.getTrajectoryFromPathPlanner(() -> true, path);

    var redEndPose = FlippingUtil.flipFieldPose(blueEndPose);

    assertNotEquals(blueTrajectory, redTrajectory);

    assertEquals(redEndPose, redTrajectory.sample(redTrajectory.getTotalTimeSeconds()).poseMeters);
  }

  @Test
  public void testResetRobotPoseAuto() {
    var pathConstraints = new PathConstraints(1, 1, 1, 1, 12);
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1, 1, Rotation2d.kZero), new Pose2d(5, 1, Rotation2d.kZero));
    var path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            new IdealStartingState(0, Rotation2d.kZero),
            new GoalEndState(0, Rotation2d.kZero));

    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().schedule(m_trajectoryUtils.generatePPHolonomicCommand(path));
    CommandScheduler.getInstance().run();

    assertEquals(path.getStartingDifferentialPose(), m_swerveDrive.getState().Pose);

    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().schedule(m_trajectoryUtils.generatePPHolonomicCommand(path));
    CommandScheduler.getInstance().run();

    assertEquals(path.flipPath().getStartingDifferentialPose(), m_swerveDrive.getState().Pose);
  }

  @Disabled("To fix")
  public void testGeneratePPHolonomicCommand() {
    var swerveRequest = new SwerveRequest.SwerveDriveBrake();
    var testDefaultCommand =
        m_swerveDrive.applyRequest(() -> swerveRequest).withName("defaultCommand");
    m_swerveDrive.setDefaultCommand(testDefaultCommand);

    var pathConstraints = new PathConstraints(1, 1, 1, 1, 12);
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1, 1, Rotation2d.kZero), new Pose2d(5, 1, Rotation2d.kZero));
    var path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            new IdealStartingState(0, Rotation2d.kZero),
            new GoalEndState(0, Rotation2d.kZero));
    CommandScheduler.getInstance().run();
    var currentCommand2 = m_swerveDrive.getCurrentCommand();

    CommandScheduler.getInstance().schedule(m_trajectoryUtils.generatePPHolonomicCommand(path));
    for (int i = 0; i < 100; i++) {
      CommandScheduler.getInstance().run();
      var currentCommand = m_swerveDrive.getCurrentCommand();
      if (currentCommand == testDefaultCommand) {
        break;
      }
    }
    assertEquals(testDefaultCommand, m_swerveDrive.getCurrentCommand());
  }
}
