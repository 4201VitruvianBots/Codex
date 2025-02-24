package org.team4201.codex.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team4201.codex.control.TalonFxTuner;

/** Mock Subsystem used for testing Codex functions using simulation. */
public class MockSimpleMotorSubsystem extends SubsystemBase {
  TalonFX motor = new TalonFX(0);
  TalonFXSimState simState = motor.getSimState();

  double GEAR_RATIO = 1.0;
  double DRUM_RADIUS = 1;
  double DRUM_ROTATIONS_TO_METERS = 2 * DRUM_RADIUS * Math.PI;
  double MIN_HEIGHT_METERS = 0;
  double MAX_HEIGHT_METERS = 1;

  ElevatorSim simModel =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          GEAR_RATIO,
          10,
          DRUM_RADIUS,
          MIN_HEIGHT_METERS,
          MAX_HEIGHT_METERS,
          true,
          0,
          0.0,
          0.0);

  TalonFxTuner tuner = new TalonFxTuner(motor, "MockSimpleMotorSubsystem Motor", this);

  /** Create an instance of {@link MockSimpleMotorSubsystem} */
  public MockSimpleMotorSubsystem() {
    setName("MockSimpleMotorSubsystem");

    SmartDashboard.putData("TEST_PID", new PIDController(1, 0, 0));
    SmartDashboard.putData("TEST", new UpDownCounter(null, null));
    SmartDashboard.putData("TEST_COMMAND", new WaitCommand(0));
  }

  @Override
  public void simulationPeriodic() {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simModel.setInputVoltage(MathUtil.clamp(simState.getMotorVoltage(), -12, 12));

    simModel.update(0.020);

    simState.setRawRotorPosition(
        simModel.getPositionMeters() * GEAR_RATIO / DRUM_ROTATIONS_TO_METERS);
    simState.setRotorVelocity(
        simModel.getVelocityMetersPerSecond() * GEAR_RATIO / DRUM_ROTATIONS_TO_METERS);
  }
}
