package org.team4201.codex.subsystems.interfaces;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.ControlRequest;
import com.fasterxml.jackson.databind.util.ArrayBuilders;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.team4201.codex.subsystems.test.Flywheel;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.stream.Stream;

public abstract class RotationalTalonFxSubsystem<
        VelocityT extends MutAngularVelocity,
        AccelerationT extends MutAngularAcceleration,
        ConfigT extends RotationalTalonFxSubsystem.Config<VelocityT, AccelerationT>,
        IoT extends RotationalTalonFxSubsystem.IO<VelocityT, AccelerationT>>
    extends BaseTalonFxSubsystem<ConfigT, IoT> {

  Type superclass = getClass().getGenericSuperclass();
  ParameterizedType paramType = (ParameterizedType) superclass;
  @SuppressWarnings("unchecked")
  private final Class<IoT> type =  (Class<IoT>) paramType.getActualTypeArguments()[0];

  public RotationalTalonFxSubsystem(ConfigT config) {
    super(config, createArray(config.motors.length));
  }

  private IoT createInstance() throws Exception {
    return type.getDeclaredConstructor().newInstance();
  }

  @SuppressWarnings("unchecked")
  private IoT[] createArray(int size) {
    return (IoT[]) Array.newInstance(type, size);
  }

  public VelocityT getSetpoint() {
    return io[0].commandedSetpoint;
  }

  public VelocityT getAppliedSetpoint() {
    return io[0].appliedSetpoint;
  }

  public void setSetpoint(VelocityT setpoint) {
    io[0].commandedSetpoint = setpoint;
    io[0].appliedSetpoint = io[0].commandedSetpoint;
    if (config.boundSetpoint) {
      io[0].appliedSetpoint =
          io[0].commandedSetpoint.gt(config.maxVelocity)
              ? config.maxVelocity
              : io[0].appliedSetpoint;
      io[0].appliedSetpoint =
          io[0].commandedSetpoint.lt(config.minVelocity)
              ? config.minVelocity
              : io[0].appliedSetpoint;
    }
  }

  public VelocityT getVelocity() {
    return io[0].currentVelocity;
  }

  public AccelerationT getAcceleration() {
    return io[0].currentAcceleration;
  }

  @Override
  protected void configureSubsystem() {}

  @Override
  public boolean atSetpoint() {
    return Math.abs(motors[0].getClosedLoopError().getValue())
        <= config.velocitySetpointThreshold.in(RotationsPerSecond);
  }

  @Override
  protected void updateIO() {
    // Update input values

    // Update command values
    // io.currentControlRequest;
  }

  public static class Config<VelocityT, AccelerationT> extends BaseTalonFxSubsystem.Config {
    public final BaseSubsystemUtils.SUBSYSTEM_TYPE subsystemType =
        BaseSubsystemUtils.SUBSYSTEM_TYPE.VELOCITY;

    public VelocityT minVelocity;
    public VelocityT maxVelocity;
    public boolean boundSetpoint = false;
    public AccelerationT minAcceleration;
    public AccelerationT maxAcceleration;

    public VelocityT velocitySetpointThreshold;

    protected Config() {
      super();
    }
  }

  public static class IO<VelocityT, AccelerationT> extends BaseTalonFxSubsystem.IO {
    public VelocityT commandedSetpoint;
    public VelocityT appliedSetpoint;
    public VelocityT currentVelocity;
    public AccelerationT currentAcceleration;
    public ControlRequest currentControlRequest;
  }
}
