package org.team4201.codex.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseTalonFxSubsystem<ConfigT extends BaseTalonFxSubsystemConfig>
    extends SubsystemBase {
  protected final ConfigT config;

  //  protected final TalonFX[] motors;

  public BaseTalonFxSubsystem(ConfigT config) {
    this.config = config;

    configureBaseSubsystem();
  }

  protected void configureBaseSubsystem() {}

  protected abstract void configureSubsystem();

  public abstract boolean atPositionSetpoint();

  public abstract boolean atVelocitySetpoint();
}
