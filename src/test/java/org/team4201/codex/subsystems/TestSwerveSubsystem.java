package org.team4201.codex.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestSwerveSubsystem {
  @BeforeEach
  public void constructDevices() {}

  @Test
  public void testSwerveSubsystemInterface() {
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

    SwerveDrive swerveDrive = new SwerveDrive(driveConstants, moduleConstants);
  }

  public static class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
      implements SwerveSubsystem {
    public SwerveDrive(
        SwerveDrivetrainConstants driveConstants,
        SwerveModuleConstants<?, ?, ?>... moduleConstants) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, driveConstants, moduleConstants);
    }

    @Override
    public RobotConfig getAutoRobotConfig() {
      return new RobotConfig(
          5, 5, new ModuleConfig(0.2, 4, 1, DCMotor.getKrakenX60(1), 4, 1), getModuleLocations());
    }

    @Override
    public PIDConstants getAutoTranslationPIDConstants() {
      return new PIDConstants(1);
    }

    @Override
    public PIDConstants getAutoRotationPIDConstants() {
      return new PIDConstants(1);
    }

    @Override
    public void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {}
  }
}
