package org.team4201.codex;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team4201.codex.simulation.FieldSim;
import org.team4201.codex.simulation.visualization.Arm2d;
import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.Flywheel2d;
import org.team4201.codex.simulation.visualization.configs.Arm2dConfig;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;
import org.team4201.codex.simulation.visualization.configs.Flywheel2dConfig;
import org.team4201.codex.subsystems.MockSimpleMotorSubsystem;
import org.team4201.codex.subsystems.MockSwerveSubsystem;
import org.team4201.codex.utils.TrajectoryUtils;

/**
 * This is NOT meant for running full robot code. It is meant for testing code that can't be easily
 * unit tested (e.g. {@link Mechanism2d}, {@link Field2d})
 */
public class Robot extends TimedRobot {
  MockSwerveSubsystem swerveDrive = new MockSwerveSubsystem();
  TrajectoryUtils trajectoryUtils = new TrajectoryUtils(swerveDrive);
  MockSimpleMotorSubsystem subsystem = new MockSimpleMotorSubsystem();
  FieldSim fieldSim = new FieldSim();

  Mechanism2d testBot = new Mechanism2d(30, 30);
  MechanismRoot2d superStructureRoot = testBot.getRoot("superStructureRoot", 15, 1);
  Elevator2d elevator2d =
      new Elevator2d(
          new Elevator2dConfig("testElevator")
              .withSuperStructureOffset(Inches.of(3))
              .withStageColors(new Color8Bit(255, 0, 0)),
          superStructureRoot);
  Arm2d arm2d = new Arm2d(new Arm2dConfig("testArm"), elevator2d.getLastStageLigament());
  Flywheel2d flywheel2d = new Flywheel2d(new Flywheel2dConfig("testFlywheel"), arm2d.getLigament());

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    if (RobotBase.isReal()) {
      DriverStation.reportError("Codex should not be run as robot code!", false);
      throw new RuntimeException("Codex is run as robot code!");
    }
    enableLiveWindowInTest(true);

    elevator2d.generateSubDisplay();
    arm2d.generateSubDisplay();
    flywheel2d.generateSubDisplay();

    SmartDashboard.putData("testBot", testBot);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    fieldSim.addPoses("robotPose", swerveDrive.getPose());
    fieldSim.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your RobotContainer class. */
  @Override
  public void autonomousInit() {
    var pathConstraints = new PathConstraints(1, 1, 1, 1, 12);
    //    var waypoints =
    //        PathPlannerPath.waypointsFromPoses(
    //            new Pose2d(1, 1, Rotation2d.kZero), new Pose2d(5, 1, Rotation2d.kZero));
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(17, 1, Rotation2d.k180deg), new Pose2d(13, 1, Rotation2d.k180deg));
    var path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            new IdealStartingState(0, Rotation2d.k180deg),
            new GoalEndState(0, Rotation2d.k180deg));

    fieldSim.addTrajectory(trajectoryUtils.getTrajectoryFromPathPlanner(() -> false, path));
    var cmd = trajectoryUtils.generatePPHolonomicCommand(path);
    CommandScheduler.getInstance().schedule(cmd);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    //        System.out.println("Elevator2d Height: " + elevator2d.getLigament().getLength());
    //        System.out.println("Arm2d Angle: " + arm2d.getLigament().getAngle());
    //        System.out.println("Flywheel2d Angle: " + flywheel2d.getLigament().getAngle());
    arm2d.update(Radians.of(Math.sin(Timer.getFPGATimestamp() % (2 * Math.PI))));
    elevator2d.update(Inches.of(Math.abs(Math.sin(Timer.getFPGATimestamp() % (2 * Math.PI))) * 12));
    flywheel2d.update(RotationsPerSecond.of(6 * Timer.getFPGATimestamp()));

    fieldSim.simulationPeriodic();
  }
}
