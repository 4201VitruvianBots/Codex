package org.team4201.codex.robot.subsystems;

public abstract class RotationalTalonFxSubsystem<
        ConfigT extends RotationalTalonFxSubsystem.Config>
    extends BaseTalonFxSubsystem<ConfigT> implements RotationalTalonFxSubsystemInterface {

  public RotationalTalonFxSubsystem(ConfigT config) {
    super(config);
  }
}
