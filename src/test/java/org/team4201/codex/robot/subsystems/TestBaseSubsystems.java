package org.team4201.codex.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team4201.codex.robot.test.Flywheel;

public class TestBaseSubsystems {
  @Test
  public void testRotationalTalonFxSubsystem() {
    var defaultConfig = Flywheel.Config.getPrimaryConfig();

    var flywheel = new Flywheel(defaultConfig);

    var test = flywheel.getAcceleration();

    var testValue = RotationsPerSecondPerSecond.of(0);

    assertEquals(testValue, test);

    System.out.println("TEST");
  }
}
