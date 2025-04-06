package org.team4201.codex.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;

public abstract class BaseDeviceInterface {
  @Getter Map<String, StatusSignal<?>> loggedSignals = new HashMap<>();

  void updateSignals() {
    BaseStatusSignal.refreshAll(loggedSignals.values().toArray(new StatusSignal<?>[0]));
  }

  public abstract void updateValues();
}
