package org.team4201.codex.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lombok.Getter;
import org.team4201.codex.robot.hardware.BaseDeviceInterface;
import org.team4201.codex.robot.hardware.LoggedTalonFX;

public abstract class BaseTalonFxSubsystem<
        ConfigT extends BaseTalonFxSubsystem.Config, IoT extends BaseTalonFxSubsystem.IO>
    extends SubsystemBase {
  @Getter protected final ConfigT defaultConfig;
  @Getter protected ConfigT config;
  @Getter protected final LoggedTalonFX[] motors;
  @Getter protected final BaseDeviceInterface[] sensors;
  @Getter protected final IoT io;

  private final List<StatusSignal<?>> signals = new ArrayList<>();

  public BaseTalonFxSubsystem(ConfigT config, IoT io) {
    this.defaultConfig = config;
    this.config = defaultConfig;
    this.motors = config.motors.toArray(new LoggedTalonFX[0]);
    this.sensors = config.sensors.toArray(new BaseDeviceInterface[0]);
    this.io = io;

    try {
      validateConfiguration();
    } catch (Exception e) {
      throw new RuntimeException(
          "[ERROR] Detected invalid configuration for BaseTalonFxSubsystem!", e);
    }

    configureBaseTalonFxSubsystem();
    configureTalonFxSubsystem();
  }

  protected void configureBaseTalonFxSubsystem() {
    for (var motor : motors) {
      signals.addAll(motor.getLoggedSignals().values());
    }
    for (var sensor : sensors) {
      signals.addAll(sensor.getLoggedSignals().values());
    }
  }

  protected void validateConfiguration() {
    // Check that the gear ratio is valid
    if (config.gearRatio <= 0) {
      throw new IllegalArgumentException("[ERROR] Gear ratio must be greater than zero!");
    }
    // Check that motors were set up
    if (config.motors.isEmpty()) {
      throw new IllegalArgumentException("[ERROR] Detected invalid Motor Setup!");
    }
    if (config.gearbox == null) {
      throw new IllegalArgumentException("[ERROR] Config.gearbox was not set!");
    }

    DCMotor testGearbox;
    if (config.motors.get(0).isProLicensed()) {
      testGearbox = DCMotor.getKrakenX60Foc(motors.length);
    } else {
      testGearbox = DCMotor.getKrakenX60(motors.length);
    }
    //       !config.gearbox.equals(DCMotor.getKrakenX44(motors.length)) &
    //       !config.gearbox.equals(DCMotor.getKrakenX44Foc(motors.length))

    // Check if the motors equals the DCMotor gearbox
    // TODO: Not implemented in WPILib 2025
    //    if (!config.gearbox.equals(testGearbox)) {
    //      throw new IllegalArgumentException("[ERROR] Detected invalid gearbox setup!");
    //    }
  }

  protected abstract void configureTalonFxSubsystem();

  private void setControlRequest(ControlRequest request) {
    io.currentControlRequest = request;
    motors[0].setControl(request);
  }

  public abstract boolean atSetpoint();

  private void updateBaseValues() {
    for (var motor : motors) {
      motor.updateValues();
    }
    for (var sensor : sensors) {
      sensor.updateValues();
    }
  }

  protected abstract void updateValues();

  @Override
  public final void periodic() {
    // Refresh all signals
    BaseStatusSignal.refreshAll(signals.toArray(new StatusSignal[0]));

    updateBaseValues();
    updateValues();
  }

  public abstract static class Config {
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

  public abstract static class IO {
    public ControlRequest currentControlRequest;
  }
}
