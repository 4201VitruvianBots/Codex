package org.team4201.codex.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;

/** Class that implements {@link SwerveSubsystem} for simulation testing */
public class MockSwerveSubsystem implements SwerveSubsystem {

  PIDConstants autoTranslationConstants = new PIDConstants(1, 0, 0);
  PIDConstants autoRotationConstants = new PIDConstants(1, 0, 0);

  Translation2d[] modulePositions = {
    new Translation2d(0.5, 0.5),
    new Translation2d(0.5, -0.5),
    new Translation2d(-0.5, 0.5),
    new Translation2d(-0.5, -0.5)
  };

  RobotConfig robotConfig =
      new RobotConfig(
          5, 5, new ModuleConfig(0.2, 4, 1, DCMotor.getKrakenX60(1), 4, 1), modulePositions);

  /** {@link MockSwerveSubsystem} constructor */
  public MockSwerveSubsystem() {}

  @Override
  public Translation2d[] getModuleLocations() {
    return modulePositions;
  }

  @Override
  public RobotConfig getAutoRobotConfig() {
    return robotConfig;
  }

  @Override
  public PIDConstants getAutoTranslationPIDConstants() {
    return autoTranslationConstants;
  }

  @Override
  public PIDConstants getAutoRotationPIDConstants() {
    return autoRotationConstants;
  }

  @Override
  public SwerveDrivetrain.SwerveDriveState getState() {
    return new SwerveDrivetrain.SwerveDriveState();
  }

  @Override
  public void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {}
}
