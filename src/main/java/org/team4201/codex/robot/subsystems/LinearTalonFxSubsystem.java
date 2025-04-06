package org.team4201.codex.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.*;

public abstract class LinearTalonFxSubsystem<
        PositionT extends MutDistance,
        VelocityT extends MutLinearVelocity,
        AccelerationT extends MutLinearAcceleration,
        ConfigT extends LinearTalonFxSubsystem.Config<PositionT, VelocityT, AccelerationT>,
        IoT extends LinearTalonFxSubsystem.IO<PositionT, VelocityT, AccelerationT>>
    extends BaseTalonFxSubsystem<ConfigT, IoT> {

  @SuppressWarnings("unchecked")
  public LinearTalonFxSubsystem(ConfigT config) {
    super(config, (IoT) new IO<PositionT, VelocityT, AccelerationT>());
  }

  @Override
  protected final void configureTalonFxSubsystem() {}

  public final PositionT getSetpoint() {
    return io.commandedSetpoint;
  }

  public final PositionT getAppliedSetpoint() {
    return io.appliedSetpoint;
  }

  public VelocityT getVelocity() {
    return io.currentVelocity;
  }

  public AccelerationT getAcceleration() {
    return io.currentAcceleration;
  }

  @Override
  public final boolean atSetpoint() {
    return Math.abs(motors[0].getClosedLoopError()) <= config.setpointThreshold.in(Meters);
  }

  public final void setSetpoint(PositionT setpoint) {
    io.commandedSetpoint = setpoint;
    io.appliedSetpoint = io.commandedSetpoint;
    if (config.boundSetpoint) {
      io.appliedSetpoint =
          io.commandedSetpoint.gt(config.maxPosition) ? config.maxPosition : io.appliedSetpoint;
      io.appliedSetpoint =
          io.commandedSetpoint.lt(config.minPosition) ? config.minPosition : io.appliedSetpoint;
    }
  }

  public static class Config<PositionT, VelocityT, AccelerationT>
      extends BaseTalonFxSubsystem.Config {
    public final BaseSubsystemUtils.SUBSYSTEM_TYPE subsystemType =
        BaseSubsystemUtils.SUBSYSTEM_TYPE.POSITION;

    public PositionT minPosition;
    public PositionT maxPosition;
    public VelocityT minVelocity;
    public VelocityT maxVelocity;
    public boolean boundSetpoint = false;
    public AccelerationT minAcceleration;
    public AccelerationT maxAcceleration;

    public PositionT setpointThreshold;

    protected Config() {
      super();
    }
  }

  public static class IO<PositionT, VelocityT, AccelerationT> extends BaseTalonFxSubsystem.IO {
    // TODO: Initialize mutable values
    //    public VelocityT commandedSetpoint = MetersPerSecond.mutable(0);
    public PositionT commandedSetpoint;
    public PositionT appliedSetpoint;
    public VelocityT currentVelocity;
    public AccelerationT currentAcceleration;
    public ControlRequest currentControlRequest;
  }
}
