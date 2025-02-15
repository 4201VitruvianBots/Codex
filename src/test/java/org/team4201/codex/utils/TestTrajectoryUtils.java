package org.team4201.codex.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team4201.codex.subsystems.TestSwerveSubsystem;

public class TestTrajectoryUtils {
  @BeforeEach
  public void constructDevices() {}

  @Test
  public void testTrajectoryUtils() {
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

    var trajectoryUtils = new TrajectoryUtils(swerveDrive);

    var pathPoints = new ArrayList<PathPoint>();
    pathPoints.add(new PathPoint(new Translation2d(0, 0)));
    pathPoints.add(new PathPoint(new Translation2d(10, 0)));

    var path =
        PathPlannerPath.fromPathPoints(
            pathPoints, new PathConstraints(1, 1, 1, 1, 12), new GoalEndState(0, new Rotation2d()));

    var wpilibTrajectory = trajectoryUtils.getTrajectoryFromPathPlanner(path);
  }
}
