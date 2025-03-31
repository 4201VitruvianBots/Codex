package org.team4201.codex.subsystems.test;

public class TestRobotContainer {
  Flywheel flywheel;

  public TestRobotContainer() {
    var defaultConfig = new Flywheel.Config();

    flywheel = new Flywheel(defaultConfig);
  }
}
