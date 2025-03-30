package org.team4201.codex.subsystems.test;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import org.team4201.codex.subsystems.interfaces.BaseTalonFxSubsystem;
import org.team4201.codex.subsystems.interfaces.SimpleTalonFxSubsystem;

public class Flywheel extends SimpleTalonFxSubsystem<BaseTalonFxSubsystem.Config> {

  public <ConfigT extends Flywheel.Config> Flywheel(ConfigT config) {
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

  public static class Config
      extends SimpleTalonFxSubsystem.Config<Angle, AngularVelocity, AngularAcceleration> {
    public Config() {
    }
  }
}
