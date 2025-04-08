package org.team4201.codex.robot.subsystems;

class BaseSubsystemUtils {
  public enum SUBSYSTEM_TYPE {
    POSITION,
    VELOCITY
  }

  public enum CONTROL_TYPE {
    NONE,
    OPEN_LOOP,
    CLOSED_LOOP_POSITION,
    CLOSED_LOOP_VELOCITY
  }
}
