package org.team4201.codex.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import lombok.Getter;
import lombok.Setter;

public class SimpleTalonFxSubsystem<ConfigT extends BaseTalonFxSubsystem.Config>
    extends BaseTalonFxSubsystem<ConfigT> {

  public SimpleTalonFxSubsystem(ConfigT config) {
    super(config);
  }

  @Override
  protected void configureSubsystem() {}

  @Override
  public boolean atPositionSetpoint() {
    return false;
  }

  @Override
  public boolean atVelocitySetpoint() {
    return false;
  }

  public static class Config<
          PositionT extends Angle,
          VelocityT extends AngularVelocity,
          AccelerationT extends AngularAcceleration>
      extends BaseTalonFxSubsystem.Config {
    public PositionT minPosition;
    public PositionT maxPosition;
    @Getter @Setter public boolean boundPosition = false;
    public VelocityT minVelocity;
    public VelocityT maxVelocity;
    public boolean boundVelocity = false;
    public AccelerationT minAcceleration;
    public AccelerationT maxAcceleration;

    public PositionT positionSetpointThreshold;
    public VelocityT velocitySetpointThreshold;
  }
}
