package org.team4201.codex.subsystems.interfaces;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseTalonFxSubsystem<ConfigT extends BaseTalonFxSubsystem.Config>
    extends SubsystemBase {
  protected final ConfigT config;

  protected final TalonFX[] motors;

  public BaseTalonFxSubsystem(ConfigT config) {
    this.config = config;
    motors = new TalonFX[] {new TalonFX(0)};

    configureBaseTalonFxSubsystem();
  }

  protected void configureBaseTalonFxSubsystem() {}

  protected abstract void configureSubsystem();

  public abstract boolean atPositionSetpoint();

  public abstract boolean atVelocitySetpoint();

  public abstract static class Config {
    public TalonFX motors;
    public int[] canIDs;
    public DCMotor gearbox;
    public  double gearRatio = 1.0;
  }
}
