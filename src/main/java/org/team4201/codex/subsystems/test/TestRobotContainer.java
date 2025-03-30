package org.team4201.codex.subsystems.test;

import org.team4201.codex.subsystems.interfaces.BaseTalonFxSubsystem;

public class TestRobotContainer {
  Flywheel flywheel;

  public TestRobotContainer() {
    var defaultConfig = new Flywheel.Config();

    flywheel = new Flywheel(defaultConfig);
  }
}
