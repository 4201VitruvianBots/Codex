package org.team4201.codex.robot.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import lombok.Getter;

public class LoggedTalonFX extends BaseDeviceInterface {
  private final TalonFX talonfx;
  @Getter private String deviceName = "";
  @Getter private boolean proLicensed;

  private final StatusSignal<Voltage> supplyVoltageSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> torqueCurrentSignal;

  private final StatusSignal<Double> dutyCycleSignal;
  private final StatusSignal<Double> referenceSignal;
  private final StatusSignal<Double> closedLoopErrorSignal;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;

  @Getter private final MutVoltage supplyVoltage;
  @Getter private final MutVoltage motorVoltage;
  @Getter private final MutCurrent supplyCurrent;
  @Getter private final MutCurrent statorCurrent;
  @Getter private final MutCurrent torqueCurrent;

  @Getter private double percentOutput;
  @Getter private double setpoint;
  @Getter private double closedLoopError;

  @Getter private final MutAngle position;
  @Getter private final MutAngularVelocity velocity;
  @Getter private final MutAngularAcceleration acceleration;

  public LoggedTalonFX(int deviceId) {
    this(deviceId, "");
  }

  public LoggedTalonFX(int deviceId, CANBus canbus) {
    this(deviceId, canbus.getName());
  }

  public LoggedTalonFX(int deviceId, String canbus) {
    this(new TalonFX(deviceId, canbus));
  }

  public LoggedTalonFX(TalonFX talonfx) {
    this.talonfx = talonfx;

    if (!this.talonfx.getDescription().isBlank()) {
      deviceName = this.talonfx.getDescription().replaceAll(" ", "_");
    } else {
      deviceName = String.format("TalonFX%02d", this.talonfx.getDeviceID());
    }

    for (int i = 0; i < 5; i++) {
      var proLicenseSignal = talonfx.getIsProLicensed();
      if (proLicenseSignal.getStatus() == StatusCode.OK) {
        proLicensed = proLicenseSignal.getValue();
        break;
      }
      if (i == 4) {
        System.out.printf("[WARN] Could not get pro license status for %s\n", deviceName);
      }
    }

    supplyVoltageSignal = this.talonfx.getSupplyVoltage(false).clone();
    motorVoltageSignal = this.talonfx.getMotorVoltage(false).clone();
    supplyCurrentSignal = this.talonfx.getSupplyCurrent(false).clone();
    statorCurrentSignal = this.talonfx.getStatorCurrent(false).clone();
    torqueCurrentSignal = this.talonfx.getTorqueCurrent(false).clone();

    dutyCycleSignal = this.talonfx.getDutyCycle(false).clone();
    referenceSignal = this.talonfx.getClosedLoopReference(false).clone();
    closedLoopErrorSignal = this.talonfx.getClosedLoopError(false).clone();

    positionSignal = this.talonfx.getPosition(false).clone();
    velocitySignal = this.talonfx.getVelocity(false).clone();
    accelerationSignal = this.talonfx.getAcceleration(false).clone();

    supplyVoltage = supplyVoltageSignal.getValue().mutableCopy();
    motorVoltage = motorVoltageSignal.getValue().mutableCopy();
    supplyCurrent = supplyCurrentSignal.getValue().mutableCopy();
    statorCurrent = statorCurrentSignal.getValue().mutableCopy();
    torqueCurrent = torqueCurrentSignal.getValue().mutableCopy();

    percentOutput = dutyCycleSignal.getValue();
    setpoint = referenceSignal.getValue();
    closedLoopError = closedLoopErrorSignal.getValue();

    position = positionSignal.getValue().mutableCopy();
    velocity = velocitySignal.getValue().mutableCopy();
    acceleration = accelerationSignal.getValue().mutableCopy();

    // Get all status signals we want to log
    loggedSignals.put("supplyVoltage", supplyVoltageSignal);
    loggedSignals.put("motorVoltage", motorVoltageSignal);
    loggedSignals.put("supplyCurrent", supplyCurrentSignal);
    loggedSignals.put("statorCurrent", statorCurrentSignal);
    loggedSignals.put("torqueCurrent", torqueCurrentSignal);

    loggedSignals.put("percentOutput", dutyCycleSignal);
    loggedSignals.put("setpoint", referenceSignal);
    loggedSignals.put("closedLoopError", closedLoopErrorSignal);

    loggedSignals.put("position", positionSignal);
    loggedSignals.put("velocity", velocitySignal);
    loggedSignals.put("acceleration", accelerationSignal);

    // TODO: Set signal refresh rates

  }

  public StatusCode setControl(ControlRequest request) {
    return talonfx.setControl(request);
  }

  @Override
  public final void updateValues() {
    supplyVoltage.mut_replace(supplyVoltageSignal.getValue());
    motorVoltage.mut_replace(motorVoltageSignal.getValue());
    supplyCurrent.mut_replace(supplyCurrentSignal.getValue());
    statorCurrent.mut_replace(statorCurrentSignal.getValue());
    torqueCurrent.mut_replace(torqueCurrentSignal.getValue());

    percentOutput = dutyCycleSignal.getValue();
    setpoint = referenceSignal.getValue();
    closedLoopError = closedLoopErrorSignal.getValue();

    position.mut_replace(positionSignal.getValue());
    velocity.mut_replace(velocitySignal.getValue());
    acceleration.mut_replace(accelerationSignal.getValue());
  }
}
