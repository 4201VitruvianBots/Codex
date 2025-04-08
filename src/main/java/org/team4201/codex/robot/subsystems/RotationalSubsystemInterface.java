package org.team4201.codex.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.*;
import lombok.Getter;
import lombok.Setter;

interface RotationalSubsystemInterface {
  Config config = new Config();

  String getName();

  MutAngularVelocity commandedSetpoint = RotationsPerSecond.mutable(0);
  MutAngularVelocity appliedSetpoint = RotationsPerSecond.mutable(0);
  MutAngularVelocity closedLoopError = RotationsPerSecond.mutable(0);
  MutAngularVelocity velocity = RotationsPerSecond.mutable(0);
  MutAngularAcceleration acceleration = RotationsPerSecondPerSecond.mutable(0);

  default boolean atSetpoint() {
    return closedLoopError.isNear(RotationsPerSecond.of(0), config.setpointThreshold);
  }

  default AngularVelocity getSetpoint() {
    return appliedSetpoint;
  }

  default AngularVelocity getDesiredSetpoint() {
    return commandedSetpoint;
  }

  default AngularVelocity getClosedLoopError() {
    return closedLoopError;
  }

  default AngularVelocity getVelocity() {
    return velocity;
  }

  default AngularAcceleration getAcceleration() {
    return acceleration;
  }

  default void setSetpoint(AngularVelocity setpoint) {
    commandedSetpoint.mut_replace(setpoint);
    appliedSetpoint.mut_replace(commandedSetpoint);
    if (config.boundSetpoint) {
      if (config.minVelocity.gte(config.maxVelocity)) {
        System.out.printf(
            "[WARN] Subsystem %s has boundSetpoint enabled, but bounds are invalid!", getName());
      }
      appliedSetpoint.mut_replace(
          commandedSetpoint.gt(config.maxVelocity) ? config.maxVelocity : appliedSetpoint);
      appliedSetpoint.mut_replace(
          commandedSetpoint.lt(config.minVelocity) ? config.minVelocity : appliedSetpoint);
    }
  }

  class Config {
    public final BaseSubsystemUtils.SUBSYSTEM_TYPE subsystemType =
        BaseSubsystemUtils.SUBSYSTEM_TYPE.VELOCITY;

    @Getter public final MutAngularVelocity minVelocity = RotationsPerSecond.mutable(0);
    @Getter public final MutAngularVelocity maxVelocity = RotationsPerSecond.mutable(0);
    @Getter @Setter public boolean boundSetpoint = false;

    @Getter
    public final MutAngularAcceleration minAcceleration = RotationsPerSecondPerSecond.mutable(0);

    @Getter
    public final MutAngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.mutable(0);

    @Getter @Setter public double setpointThreshold;

    protected Config() {
      super();
    }

    void setMinVelocity(AngularVelocity velocity) {
      minVelocity.mut_replace(velocity);
    }

    void setMaxVelocity(AngularVelocity velocity) {
      maxVelocity.mut_replace(velocity);
    }

    void setMinAcceleration(AngularAcceleration acceleration) {
      minAcceleration.mut_replace(acceleration);
    }

    void setMaxAcceleration(AngularAcceleration acceleration) {
      maxAcceleration.mut_replace(acceleration);
    }
  }
}
