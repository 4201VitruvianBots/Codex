package org.team4201.codex.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;

public abstract class RotationalTalonFxSubsystem<
        VelocityT extends MutAngularVelocity,
        AccelerationT extends MutAngularAcceleration,
        ConfigT extends RotationalTalonFxSubsystem.Config<VelocityT, AccelerationT>,
        IoT extends RotationalTalonFxSubsystem.IO<VelocityT, AccelerationT>>
    extends BaseTalonFxSubsystem<ConfigT, IoT> {

  @SuppressWarnings("unchecked")
  public RotationalTalonFxSubsystem(ConfigT config) {
    super(config, (IoT) new IO<VelocityT, AccelerationT>());
  }

  @Override
  protected final void configureTalonFxSubsystem() {}

  public final VelocityT getSetpoint() {
    return io.commandedSetpoint;
  }

  public final VelocityT getAppliedSetpoint() {
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
    return Math.abs(motors[0].getClosedLoopError())
        <= config.setpointThreshold.in(RotationsPerSecond);
  }

  public final void setSetpoint(VelocityT setpoint) {
    io.commandedSetpoint = setpoint;
    io.appliedSetpoint = io.commandedSetpoint;
    if (config.boundSetpoint) {
      io.appliedSetpoint =
          io.commandedSetpoint.gt(config.maxVelocity) ? config.maxVelocity : io.appliedSetpoint;
      io.appliedSetpoint =
          io.commandedSetpoint.lt(config.minVelocity) ? config.minVelocity : io.appliedSetpoint;
    }
  }

  @Override
  protected final void updateValues() {
    //    io.currentAcceleration =
  }

  public static class Config<VelocityT, AccelerationT> extends BaseTalonFxSubsystem.Config {
    public final BaseSubsystemUtils.SUBSYSTEM_TYPE subsystemType =
        BaseSubsystemUtils.SUBSYSTEM_TYPE.VELOCITY;

    public VelocityT minVelocity;
    public VelocityT maxVelocity;
    public boolean boundSetpoint = false;
    public AccelerationT minAcceleration;
    public AccelerationT maxAcceleration;

    public VelocityT setpointThreshold;

    protected Config() {
      super();
    }
  }

  @SuppressWarnings("unchecked")
  public static class IO<VelocityT, AccelerationT> extends BaseTalonFxSubsystem.IO {
    public VelocityT commandedSetpoint = (VelocityT) RotationsPerSecond.mutable(0);
    public VelocityT appliedSetpoint = (VelocityT) RotationsPerSecond.mutable(0);

    public VelocityT currentVelocity = (VelocityT) RotationsPerSecond.mutable(0);
    public AccelerationT currentAcceleration =
        (AccelerationT) RotationsPerSecondPerSecond.mutable(0);
    public ControlRequest currentControlRequest = new DutyCycleOut(0);
  }
}
