package org.team4201.codex.control;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import org.team4201.codex.utils.CtreUtils;

/** Class to interactively tune a {@link TalonFX} motor using NetworkTables. */
public class TalonFxTuner implements Sendable {
  /** Enumerated CTRE {@link ControlRequest} */
  public enum CONTROL_TYPE {
    /** DutyCycleOut */
    DutyCycleOut(0),
    /** PositionVoltage */
    PositionVoltage(1),
    /** VelocityVoltage */
    VelocityVoltage(2),
    /** MotionMagicVoltage */
    MotionMagicVoltage(1),
    /** MotionMagicVelocityVoltage */
    MotionMagicVelocityVoltage(2),
    /** TorqueCurrentFOC */
    TorqueCurrentFOC(0),
    /** PositionTorqueCurrentFOC */
    PositionTorqueCurrentFOC(1),
    /** VelocityTorqueCurrentFOC */
    VelocityTorqueCurrentFOC(2),
    /** MotionMagicTorqueCurrentFOC */
    MotionMagicTorqueCurrentFOC(1),
    /** MotionMagicVelocityTorqueCurrentFOC */
    MotionMagicVelocityTorqueCurrentFOC(2);

    final int type;

    CONTROL_TYPE(final int type) {
      this.type = type;
    }
  }

  Subsystem subsystem;
  Command subsystemDefaultCommand;
  TalonFX motor;
  TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
  TalonFXConfiguration currentConfig;
  String name;

  NetworkTableInstance instance = NetworkTableInstance.getDefault();
  NetworkTable controlTable;
  private final Map<String, DoubleSubscriber> ntDoubleSubs = new HashMap<>();
  private final Map<String, Double> lastDoubleValues = new HashMap<>();
  boolean configsApplied;
  String lastAppliedTime = "00:00:00";

  private DoubleSubscriber setpointSub;
  private final SendableChooser<CONTROL_TYPE> controlTypeSelector = new SendableChooser<>();

  private Function<Double, Angle> doubleToAngle = Degree::of;
  private Function<Double, AngularVelocity> doubleToAngularVelocity = RadiansPerSecond::of;
  private boolean setpointApplied;

  /**
   * Create an instance of {@link TalonFxTuner} for a given {@link TalonFX}.
   *
   * @param motor TalonFX motor to control
   */
  public TalonFxTuner(TalonFX motor) {
    this(motor, motor.getDescription(), null);
  }

  /**
   * Create an instance of {@link TalonFxTuner} for a given {@link TalonFX}.
   *
   * @param motor TalonFX motor to control
   * @param subsystem The subsystem the TalonFX motor is part of (Default is null)
   */
  public TalonFxTuner(TalonFX motor, Subsystem subsystem) {
    this(motor, motor.getDescription(), subsystem);
  }

  /**
   * Create an instance of {@link TalonFxTuner} for a given {@link TalonFX}.
   *
   * @param motor TalonFX motor to control
   * @param name Custom name (Default name is the motor's description)
   */
  public TalonFxTuner(TalonFX motor, String name) {
    this(motor, name, null);
  }

  /**
   * Create an instance of {@link TalonFxTuner} for a given {@link TalonFX}.
   *
   * @param motor TalonFX motor to control
   * @param name Custom name (Default name is the motor's description)
   * @param subsystem The subsystem the TalonFX motor is part of (Default is null)
   */
  public TalonFxTuner(TalonFX motor, String name, Subsystem subsystem) {
    this.motor = motor;
    this.name = name.replace(" ", "_");
    this.subsystem = subsystem;
    controlTable = instance.getTable("Control").getSubTable(this.name);

    this.motor.getConfigurator().refresh(defaultConfig);
    this.currentConfig = defaultConfig;

    ntDoubleSubs.put(
        "minOut",
        controlTable
            .getDoubleTopic("minOut")
            .subscribe(defaultConfig.MotorOutput.PeakReverseDutyCycle));
    ntDoubleSubs.put(
        "maxOut",
        controlTable
            .getDoubleTopic("maxOut")
            .subscribe(defaultConfig.MotorOutput.PeakForwardDutyCycle));

    ntDoubleSubs.put("kP", controlTable.getDoubleTopic("kP").subscribe(defaultConfig.Slot0.kP));
    ntDoubleSubs.put("kI", controlTable.getDoubleTopic("kI").subscribe(defaultConfig.Slot0.kI));
    ntDoubleSubs.put("kD", controlTable.getDoubleTopic("kD").subscribe(defaultConfig.Slot0.kD));
    ntDoubleSubs.put("kG", controlTable.getDoubleTopic("kG").subscribe(defaultConfig.Slot0.kG));
    ntDoubleSubs.put("kS", controlTable.getDoubleTopic("kS").subscribe(defaultConfig.Slot0.kS));
    ntDoubleSubs.put("kV", controlTable.getDoubleTopic("kV").subscribe(defaultConfig.Slot0.kV));
    ntDoubleSubs.put("kA", controlTable.getDoubleTopic("kA").subscribe(defaultConfig.Slot0.kA));

    ntDoubleSubs.put(
        "MMv",
        controlTable
            .getDoubleTopic("MMv")
            .subscribe(defaultConfig.MotionMagic.MotionMagicCruiseVelocity));
    ntDoubleSubs.put(
        "MMa",
        controlTable
            .getDoubleTopic("MMa")
            .subscribe(defaultConfig.MotionMagic.MotionMagicAcceleration));
    ntDoubleSubs.put(
        "MMj",
        controlTable.getDoubleTopic("MMv").subscribe(defaultConfig.MotionMagic.MotionMagicJerk));

    setpointSub = controlTable.getDoubleTopic("setpoint").subscribe(0.0);

    for (var v : CONTROL_TYPE.values()) {
      controlTypeSelector.addOption(v.name(), v);
    }

    SendableRegistry.add(this, name.substring(name.lastIndexOf('.') + 1));

    // Add this to NetworkTables
    SendableBuilderImpl builderImpl = new SendableBuilderImpl();
    builderImpl.setTable(controlTable);
    initSendable(builderImpl);

    if (this.subsystem != null) {
      subsystemDefaultCommand = this.subsystem.getDefaultCommand();
    }
  }

  /**
   * Function to override the motor's {@link Subsystem}'s default command or return it to its
   * original value if the motor is associated with a subsystem.
   *
   * @param useTuner Overwrite the subsystem's default command, or return it to its original value
   */
  public void updateSubsystemDefaultCommand(boolean useTuner) {
    if (subsystem == null) {
      DriverStation.reportWarning("[TalonFxTuner] This motor does not have a subsystem!", false);
      return;
    }
    if (subsystemDefaultCommand == null) {
      DriverStation.reportWarning(
          "[TalonFxTuner] This motor's subsystem does not have a default command!", false);
      return;
    }

    if (useTuner) {
      subsystem.setDefaultCommand(new WaitCommand(0));
    } else {
      subsystem.setDefaultCommand(subsystemDefaultCommand);
    }
  }

  private void setConfigsApplied(boolean value) {
    if (value) {
      lastAppliedTime = LocalDateTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss"));
    }
    configsApplied = value;
  }

  private boolean areConfigsApplied() {
    return configsApplied;
  }

  private String getLastAppliedTime() {
    return lastAppliedTime;
  }

  private void resetToDefaults() {
    applyConfigs(defaultConfig);
  }

  private double get(String key) {
    return lastDoubleValues.get(key);
  }

  private void set(String key, double value) {
    lastDoubleValues.put(key, value);
  }

  private void applyConfigsFromNt() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.PeakReverseDutyCycle = lastDoubleValues.get("minOut");
    config.MotorOutput.PeakForwardDutyCycle = lastDoubleValues.get("maxOut");

    config.Slot0.kP = lastDoubleValues.get("kP");
    config.Slot0.kI = lastDoubleValues.get("kI");
    config.Slot0.kD = lastDoubleValues.get("kD");
    config.Slot0.kG = lastDoubleValues.get("kG");
    config.Slot0.kS = lastDoubleValues.get("kS");
    config.Slot0.kV = lastDoubleValues.get("kV");
    config.Slot0.kA = lastDoubleValues.get("kA");

    config.MotionMagic.MotionMagicCruiseVelocity = lastDoubleValues.get("MMv");
    config.MotionMagic.MotionMagicAcceleration = lastDoubleValues.get("MMa");
    config.MotionMagic.MotionMagicJerk = lastDoubleValues.get("MMj");

    applyConfigs(config);
  }

  private void applyConfigs(TalonFXConfiguration config) {
    CtreUtils.configureTalonFx(motor, config);
    setConfigsApplied(true);
  }

  private void setSetpointApplied(boolean value) {
    setpointApplied = value;
  }

  private boolean isSetpointApplied() {
    return setpointApplied;
  }

  /**
   * Set a function to convert from the input double from the dashboard to the expected motor
   * setpoint as an {@link Angle}
   *
   * @param function Function to convert from a double to the TalonFX Angle setpoint
   */
  public void setPositionSetpointConversionFactor(Function<Double, Angle> function) {
    doubleToAngle = function;
  }

  /**
   * Set a function to convert from the input double from the dashboard to the expected motor
   * setpoint as an {@link AngularVelocity}
   *
   * @param function Function to convert from a double to the TalonFX AngularVelocity setpoint
   */
  public void setVelocitySetpointConversionFactor(Function<Double, AngularVelocity> function) {
    doubleToAngularVelocity = function;
  }

  private void updateControlRequest(CONTROL_TYPE controlType) {}

  private void applySetpoint() {
    var controlType = controlTypeSelector.getSelected();
    var setpoint = setpointSub.get();

    ControlRequest request = new NeutralOut();
    switch (controlType) {
      case DutyCycleOut -> request = new DutyCycleOut(setpoint);
      case PositionVoltage -> request = new PositionVoltage(doubleToAngle.apply(setpoint));
      case VelocityVoltage ->
          request = new VelocityVoltage(doubleToAngularVelocity.apply(setpoint));
      case MotionMagicVoltage -> request = new MotionMagicVoltage(doubleToAngle.apply(setpoint));
      case MotionMagicVelocityVoltage ->
          request = new MotionMagicVelocityVoltage(doubleToAngularVelocity.apply(setpoint));
      case TorqueCurrentFOC -> request = new TorqueCurrentFOC(setpoint);
      case PositionTorqueCurrentFOC ->
          request = new PositionTorqueCurrentFOC(doubleToAngle.apply(setpoint));
      case VelocityTorqueCurrentFOC ->
          request = new VelocityTorqueCurrentFOC(doubleToAngularVelocity.apply(setpoint));
      case MotionMagicTorqueCurrentFOC ->
          request = new MotionMagicTorqueCurrentFOC(doubleToAngle.apply(setpoint));
      case MotionMagicVelocityTorqueCurrentFOC ->
          request =
              new MotionMagicVelocityTorqueCurrentFOC(doubleToAngularVelocity.apply(setpoint));
    }

    motor.setControl(request);
    setSetpointApplied(true);
  }

  /** Replicate {@link Command} initSendable() */
  @Override
  public void initSendable(SendableBuilder builder) {
    // Add a selector to choose between TalonFX ControlRequests
    //    controlTypeSelector.initSendable(builder);

    builder.setSmartDashboardType("LW Subsystem");
    builder.setActuator(true);
    builder.setSafeState(motor::stopMotor);
    for (var sub : ntDoubleSubs.entrySet()) {
      builder.addDoubleProperty(
          sub.getKey(), () -> this.get(sub.getKey()), (d) -> this.set(sub.getKey(), d));
    }
    builder.addStringProperty(".name", () -> name, null);

    builder.addBooleanProperty(
        "running",
        this::areConfigsApplied,
        value -> {
          if (value) {
            resetToDefaults();
          }
        });
    builder.addBooleanProperty(
        "Set Setpoint",
        this::isSetpointApplied,
        value -> {
          if (isSetpointApplied()) {
            setSetpointApplied(false);
            applySetpoint();
          }
        });
    builder.addStringProperty("LastUpdateTime", this::getLastAppliedTime, null);
  }

  private void periodic() {
    boolean updated = false;
    for (var entry : ntDoubleSubs.entrySet()) {
      var lastValue = lastDoubleValues.get(entry.getKey());
      if (lastValue != entry.getValue().get()) {
        lastDoubleValues.put(entry.getKey(), entry.getValue().get());
        updated = true;
      }
    }
    if (updated) {
      applyConfigsFromNt();
    }
  }
}
