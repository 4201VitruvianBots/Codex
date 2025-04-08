package org.team4201.codex.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lombok.Getter;
import org.team4201.codex.robot.hardware.BaseDeviceInterface;
import org.team4201.codex.robot.hardware.LoggedTalonFX;

interface BaseTalonFxSubsystemInterface {

  default void updateBaseValues() {}

  default void updateSubsystemValues() {}

  default void userPeriodic() {}

  class Config {
    @Getter public List<LoggedTalonFX> motors = new ArrayList<>();
    @Getter public List<BaseDeviceInterface> sensors = new ArrayList<>();
    @Getter public DCMotor gearbox;
    @Getter public double gearRatio = 1.0;

    protected Config() {}

    public BaseTalonFxSubsystem.Config withMotors(TalonFX... motors) {
      this.motors.addAll(Arrays.stream(motors).map(LoggedTalonFX::new).toList());
      return this;
    }

    public BaseTalonFxSubsystem.Config withMotors(LoggedTalonFX... motors) {
      this.motors.addAll(List.of(motors));
      return this;
    }

    public BaseTalonFxSubsystem.Config withSensors(BaseDeviceInterface... sensors) {
      this.sensors.addAll(List.of(sensors));
      return this;
    }
  }

  class IO {
    public ControlRequest currentControlRequest;
    public BaseSubsystemUtils.CONTROL_TYPE currentControlType;
  }
}
