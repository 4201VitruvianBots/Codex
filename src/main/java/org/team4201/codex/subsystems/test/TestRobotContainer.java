package org.team4201.codex.subsystems.test;

public class TestRobotContainer {
  Flywheel flywheel;

  public TestRobotContainer() {
    var defaultConfig = Flywheel.Config.getPrimaryConfig();

    flywheel = new Flywheel(defaultConfig);
  }
}
