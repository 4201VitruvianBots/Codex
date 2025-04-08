package org.team4201.codex.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import org.team4201.codex.robot.hardware.BaseDeviceInterface;
import org.team4201.codex.robot.hardware.LoggedTalonFX;

abstract class BaseTalonFxSubsystem<ConfigT extends BaseTalonFxSubsystemInterface.Config>
    extends SubsystemBase implements BaseTalonFxSubsystemInterface {
  @Getter private final ConfigT defaultConfig;
  @Getter private ConfigT config;
  @Getter private final List<LoggedTalonFX> motors = new ArrayList<>();
  @Getter private final List<BaseDeviceInterface> sensors = new ArrayList<>();
  @Getter private final IO io = new IO();

  private final List<StatusSignal<?>> signals = new ArrayList<>();

  public BaseTalonFxSubsystem(ConfigT config) {
    this.defaultConfig = config;
    this.config = defaultConfig;
    this.motors.addAll(config.motors);
    this.sensors.addAll(config.sensors);

    try {
      validateConfiguration();
    } catch (Exception e) {
      throw new RuntimeException(
          "[ERROR] Detected invalid configuration for BaseTalonFxSubsystem!", e);
    }

    configureTalonFxSubsystem();
  }

  private void configureTalonFxSubsystem() {
    for (var motor : motors) {
      signals.addAll(motor.getLoggedSignals().values());
    }
    for (var sensor : sensors) {
      signals.addAll(sensor.getLoggedSignals().values());
    }
  }

  private void validateConfiguration() {
    // Check that the gear ratio is valid
    if (config.gearRatio <= 0) {
      throw new IllegalArgumentException("[ERROR] Gear ratio must be greater than zero!");
    }
    // Check that motors were set up
    if (config.motors.isEmpty()) {
      throw new IllegalArgumentException("[ERROR] Detected invalid motor Setup!");
    }
    if (config.gearbox == null) {
      throw new IllegalArgumentException("[ERROR] Config.gearbox was not set!");
    }

    DCMotor testGearbox;
    if (config.motors.get(0).isProLicensed()) {
      testGearbox = DCMotor.getKrakenX60Foc(motors.size());
    } else {
      testGearbox = DCMotor.getKrakenX60(motors.size());
    }
    //       !config.gearbox.equals(DCMotor.getKrakenX44(motors.length)) &
    //       !config.gearbox.equals(DCMotor.getKrakenX44Foc(motors.length))

    // Check if the motors equals the DCMotor gearbox
    // TODO: Not implemented in WPILib 2025
    //    if (!config.gearbox.equals(testGearbox)) {
    //      throw new IllegalArgumentException("[ERROR] Detected invalid gearbox setup!");
    //    }
  }

  private void resetConfig() {
    config = defaultConfig;
  }

  private void setControlRequest(ControlRequest request) {
    io.currentControlRequest = request;
  }

  public final void updateBaseValues() {
    for (var motor : motors) {
      motor.updateValues();
    }
    for (var sensor : sensors) {
      sensor.updateValues();
    }

    motors.get(0).setControl(io.currentControlRequest);
  }

  @Override
  public final void periodic() {
    // Refresh all signals
    BaseStatusSignal.refreshAll(signals.toArray(new StatusSignal[0]));

    updateBaseValues();

    updateSubsystemValues();

    userPeriodic();
  }
}
