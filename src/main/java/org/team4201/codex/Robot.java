package org.team4201.codex;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team4201.codex.simulation.FieldSim;
import org.team4201.codex.simulation.visualization.Arm2d;
import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.Flywheel2d;
import org.team4201.codex.simulation.visualization.configs.Arm2dConfig;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;
import org.team4201.codex.simulation.visualization.configs.Flywheel2dConfig;

import static edu.wpi.first.units.Units.*;

/**
 * This is NOT meant for running full robot code. It is meant for testing code that can't be easily unit tested
 * (e.g. {@link Mechanism2d}, {@link Field2d})
 *
 */
public class Robot extends TimedRobot {
    FieldSim fieldSim = new FieldSim();


    Mechanism2d testBot = new Mechanism2d(30, 30);
    MechanismRoot2d superStructureRoot = testBot.getRoot("superStructureRoot", 15, 1);
    Elevator2d elevator2d = new Elevator2d(new Elevator2dConfig("testElevator"), superStructureRoot);
    Arm2d arm2d = new Arm2d(new Arm2dConfig("testArm"), elevator2d.getLigament());
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

        Mechanism2d elevatorMechanism = new Mechanism2d(10, 30);
        elevator2d.setAngle(Degree.of(90));
        elevatorMechanism.getRoot("elevatorRoot", 5, 1).append(elevator2d.getSubElevator().getLigament());

        Mechanism2d armMechanism = new Mechanism2d(15, 15);
        arm2d.setAngle(Degree.of(90));
        arm2d.getArmSubMechanism().getConfig().setAngleOffset(Degree.of(90));
        armMechanism.getRoot("armRoot", 7.5, 1).append(arm2d.getArmSubMechanism().getLigament());

        Mechanism2d flywheelMechanism = new Mechanism2d(5, 5);
        flywheelMechanism.getRoot("flywheelRoot", 2.5, 2.5).append(flywheel2d.getSubFlywheel().getLigament());

        SmartDashboard.putData("testBot", testBot);
        SmartDashboard.putData("elevator", elevatorMechanism);
        SmartDashboard.putData("arm", armMechanism);
        SmartDashboard.putData("flywheel", flywheelMechanism);
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
        fieldSim.simulationPeriodic();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your RobotContainer class. */
    @Override
    public void autonomousInit() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
    }

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
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        System.out.println("Elevator2d Height: " + elevator2d.getLigament().getLength());
        System.out.println("Arm2d Angle: " + arm2d.getLigament().getAngle());
        System.out.println("Flywheel2d Angle: " + flywheel2d.getLigament().getAngle());
        arm2d.update(Radians.of(Math.sin(Timer.getFPGATimestamp() %  (2 * Math.PI))));
        elevator2d.update(Inches.of(Math.abs(Math.sin(Timer.getFPGATimestamp() % (2 * Math.PI))) * 24));
        flywheel2d.update(RotationsPerSecond.of(6 * Timer.getFPGATimestamp()));
    }
}
