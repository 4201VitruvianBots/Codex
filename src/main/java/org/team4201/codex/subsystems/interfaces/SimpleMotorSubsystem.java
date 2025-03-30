package org.team4201.codex.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class SimpleMotorSubsystem extends BaseTalonFxSubsystem<SimpleMotorSubsystem.Config> {

  public SimpleMotorSubsystem(Config config) {
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
      extends BaseTalonFxSubsystemConfig<Angle, AngularVelocity, AngularAcceleration> {}

  public static Config buildConfig() {
    var config = new Config();
    config.gearRatio = 2.0;

    return config;
  }
}
