package org.team4201.codex.subsystems.interfaces;

import static java.util.Map.entry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;

public abstract class BaseTalonFxSubsystem<
        ConfigT extends BaseTalonFxSubsystem.Config, IoT extends BaseTalonFxSubsystem.IO>
    extends SubsystemBase {
  @Getter protected final ConfigT defaultConfig;
  @Getter protected ConfigT config;
  @Getter protected final TalonFX[] motors;
  @Getter protected final IoT[] io;

  private final Map<String, StatusSignal<?>> signalMap = new HashMap<>();
  private final Map<String, String> loggedSignals =
      Map.ofEntries(
          entry("supplyVoltage", "getSupplyVoltage"),
          entry("motorVoltage", "getMotorVoltage"),
          entry("supplyCurrent", "getSupplyCurrent"),
          entry("statorCurrent", "getStatorCurrent"),
          entry("torqueCurrent", "getTorqueCurrent"),
          entry("percentOutput", "getDutyCycleOut"),
          entry("setpoint", "getReference"),
          entry("position", "getPosition"),
          entry("velocity", "getVelocity"),
          entry("acceleration", "getAcceleration"));

  public BaseTalonFxSubsystem(ConfigT config, IoT[] io) {
    this.defaultConfig = config;
    this.config = defaultConfig;
    this.motors = config.motors;
    this.io = io;

    if (!validateConfiguration()) {
      throw new IllegalArgumentException(
          "[ERROR] Detected invalid configuration for BaseTalonFxSubsystem!");
    }

    configureBaseTalonFxSubsystem();
  }

  protected void configureBaseTalonFxSubsystem() {
    for (TalonFX motor : motors) {
      var motorPrefix = "TalonFX" + motor.getDeviceID() + "_";

      for (var loggedSignal : loggedSignals.entrySet()) {
        try {
          var statusSignal =
              (StatusSignal<?>) motor.getClass().getMethod(loggedSignal.getValue()).invoke(motor);
          signalMap.put(motorPrefix + loggedSignal.getValue(), statusSignal);
        } catch (NoSuchMethodException e) {
          System.out.printf(
              "[WARN] TalonFX.%s() is not a valid function!\n", loggedSignal.getValue());
        } catch (InvocationTargetException e) {
          System.out.printf(
              "[ERROR] TalonFX.%s() threw an exception!\n%s\n",
              loggedSignal.getValue(), e.getCause());
        } catch (IllegalAccessException e) {
          System.out.printf("[WARN] Illegal access TalonFX.%s()!\n", loggedSignal.getValue());
        }
      }
    }
  }

  protected boolean validateConfiguration() {
    // Check that the gear ratio is valid
    if (config.gearRatio <= 0) {
      System.out.println("[ERROR] Gear Ratio must be greater than zero!");
      return false;
    }
    // Check that motors were set up
    if (config.motors.length != 0) {
      System.out.println("[ERROR] Detected Invalid Motor Setup!");
      return false;
    }
    // Check if the motors equals the DCMotor gearbox
    if (!config.gearbox.equals(DCMotor.getKrakenX60(motors.length))
        || !config.gearbox.equals(DCMotor.getKrakenX60Foc(motors.length))) {
      //       !config.gearbox.equals(DCMotor.getKrakenX44(motors.length)) ||
      //       !config.gearbox.equals(DCMotor.getKrakenX44Foc(motors.length))
      System.out.println("[ERROR] Detected Invalid Motor Setup!");
      return false;
    }

    return true;
  }

  protected abstract void configureSubsystem();

  public abstract boolean atSetpoint();

  protected abstract void updateIO();

  private void updateSignals() {
    for (int i = 0; i < motors.length; i++) {
      var motorPrefix = "TalonFX" + motors[i].getDeviceID() + "_";
      io[i].percentOutput = signalMap.get(motorPrefix + "percentOutput").getValueAsDouble();
      // io[i].percentOutput = signalMap.get(motorPrefix + "percentOutput").getValueAsDouble();
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(signalMap.values().toArray(new StatusSignal<?>[0]));
    updateSignals();
    updateIO();
  }

  public abstract static class Config {
    @Getter public TalonFX[] motors;
    @Getter public DCMotor gearbox;
    @Getter public double gearRatio = 1.0;

    protected Config() {}
  }

  public abstract static class IO {
    public double percentOutput;
  }
}
