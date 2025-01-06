package org.team4201.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>, Subsystem {

    ChassisSpeeds getChassisSpeeds();

    void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds);
}
