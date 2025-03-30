package org.team4201.codex.subsystems.interfaces;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import lombok.Getter;
import lombok.Setter;

public abstract class BaseTalonFxSubsystemConfig<
    PositionT extends Measure<?>, VelocityT extends Measure<?>, AccelerationT extends Measure<?>> {

  TalonFX motors;
  TalonFXConfiguration configs;
  int[] canIDs;
  DCMotor gearbox;
  double gearRatio = 1.0;

  PositionT minPosition;
  PositionT maxPosition;
  @Getter @Setter boolean boundPosition = false;
  VelocityT minVelocity;
  VelocityT maxVelocity;
  boolean boundVelocity = false;
  AccelerationT minAcceleration;
  AccelerationT maxAcceleration;

  PositionT positionSetpointThreshold;
  VelocityT velocitySetpointThreshold;

  void withCanIds(int[] canIDs) {
    this.canIDs = canIDs;
  }
}
