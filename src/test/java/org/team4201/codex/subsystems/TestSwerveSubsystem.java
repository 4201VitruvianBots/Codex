package org.team4201.codex.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;

public class TestSwerveSubsystem {
    @BeforeEach
    public void constructDevices() {

    }

    public void testSwerveSubsystemInterface() {
        SwerveDrivetrainConstants driveConstants = new SwerveDrivetrainConstants();
        SwerveModuleConstants moduleConstants = new SwerveModuleConstants();

        SwerveDrive m_swerveDrive = new SwerveDrive(
                driveConstants, moduleConstants, moduleConstants, moduleConstants, moduleConstants);
    }

    private class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveSubsystem {
        public SwerveDrive(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, driveConstants, moduleConstants);
        }

        @Override
        public void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {

        }
    }
}
