package org.team4201.codex.subsystems.interfaces;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class RotationalTalonFxSubsystem<
        VelocityT extends AngularVelocity,
        AccelerationT extends AngularAcceleration,
        ConfigT extends RotationalTalonFxSubsystem.Config<VelocityT, AccelerationT>>
    extends BaseTalonFxSubsystem<ConfigT> {
  private final IO<VelocityT, AccelerationT> io = new IO<>();

  public RotationalTalonFxSubsystem(ConfigT config) {
    super(config);
  }

  public VelocityT getSetpoint() {
    return io.commandedSetpoint;
  }

  public VelocityT getAppliedSetpoint() {
    return io.appliedSetpoint;
  }

  public void setSetpoint(VelocityT setpoint) {
    io.commandedSetpoint = setpoint;
    io.appliedSetpoint = io.commandedSetpoint;
    if (config.boundSetpoint) {
      io.appliedSetpoint =
          io.commandedSetpoint.gt(config.maxVelocity) ? config.maxVelocity : io.appliedSetpoint;
      io.appliedSetpoint =
          io.commandedSetpoint.lt(config.minVelocity) ? config.minVelocity : io.appliedSetpoint;
    }
  }

  public VelocityT getVelocity() {
    return io.currentVelocity;
  }

  public AccelerationT getAcceleration() {
    return io.currentAcceleration;
  }

  @Override
  protected void configureSubsystem() {}

  @Override
  public boolean atSetpoint() {
    return Math.abs(motors[0].getClosedLoopError().getValue())
        <= config.velocitySetpointThreshold.in(RotationsPerSecond);
  }

  @Override
  protected void updateValues() {}

  public abstract static class Config<VelocityT, AccelerationT>
      extends BaseTalonFxSubsystem.Config {
    public final BaseSubsystemUtils.SUBSYSTEM_TYPE subsystemType =
        BaseSubsystemUtils.SUBSYSTEM_TYPE.VELOCITY;

    public VelocityT minVelocity;
    public VelocityT maxVelocity;
    public boolean boundSetpoint = false;
    public AccelerationT minAcceleration;
    public AccelerationT maxAcceleration;

    public VelocityT velocitySetpointThreshold;
  }

  private static class IO<VelocityT, AccelerationT> extends BaseTalonFxSubsystem.BaseIO {
    public VelocityT commandedSetpoint;
    public VelocityT appliedSetpoint;
    public VelocityT currentVelocity;
    public AccelerationT currentAcceleration;
  }
}
