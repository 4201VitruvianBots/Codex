package org.team4201.codex.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import lombok.Getter;
import org.team4201.codex.robot.hardware.BaseDeviceInterface;
import org.team4201.codex.robot.hardware.LoggedTalonFX;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public interface RotationalTalonFxSubsystemInterface extends RotationalSubsystemInterface {
  Config config = new Config();

  class Config extends RotationalSubsystemInterface.Config {
      @Getter public List<LoggedTalonFX> motors = new ArrayList<>();
      @Getter public List<BaseDeviceInterface> sensors = new ArrayList<>();
      @Getter public DCMotor gearbox;
      @Getter public double gearRatio = 1.0;

      protected Config() {}

      public Config withMotors(TalonFX... motors) {
        this.motors.addAll(Arrays.stream(motors).map(LoggedTalonFX::new).toList());
        return this;
      }

      public Config withMotors(LoggedTalonFX... motors) {
        this.motors.addAll(List.of(motors));
        return this;
      }

      public Config withSensors(BaseDeviceInterface... sensors) {
        this.sensors.addAll(List.of(sensors));
        return this;
      }
  }
}
