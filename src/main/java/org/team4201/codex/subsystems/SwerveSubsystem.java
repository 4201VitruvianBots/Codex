package org.team4201.codex.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Generic interface that extends the WPILib Subsystem to incorporate functions for PathPlanner.
 *
 */
public interface SwerveSubsystem extends Subsystem {

    /**
     * Function to get the CTRE SwerveDriveState. Used to get the Robot's {@link Pose2d}.
     *
     * @return SwerveDriveState
     */
    SwerveDriveState getState();

    /**
     * Function to get the {@link ChassisSpeeds} for PathPlanner's path following.
     *
     * @return ChassisSpeeds
     */
    ChassisSpeeds getChassisSpeeds();

    /**
     * Function for PathPlanner to control the robot's motion in auto.
     *
     * @param chassisSpeeds WPILib's {@link ChassisSpeeds}
     * @param feedforwards PathPlanner's {@link DriveFeedforwards}
     */
    void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards);
}
