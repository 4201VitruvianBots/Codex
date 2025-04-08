package org.team4201.codex.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team4201.codex.robot.test.Flywheel;

public class TestBaseSubsystems {
  @Test
  public void testRotationalTalonFxSubsystem() {
    var defaultConfig = Flywheel.Config.getPrimaryConfig();

    var flywheel = new Flywheel(defaultConfig);

    // Test common get/set functions
    flywheel.setSetpoint(RotationsPerSecond.of(2));
    assertEquals(flywheel.getSetpoint(), DegreesPerSecond.of(720));

    assertEquals(RotationsPerSecond.of(0), flywheel.getVelocity());
    assertEquals(RotationsPerSecondPerSecond.of(0), flywheel.getAcceleration());

    // Variables/Functions that should not be accessible
    //    var io = flywheel.io;
    //    var config = flywheel.config;
    //    var motors = flywheel.motors;
    //    var sensors = flywheel.sensors;
    flywheel.periodic(); // Maybe?

    //    flywheel.updateValues();
    //    flywheel.configureBaseTalonFxSubsystem();
    //    flywheel.validateConfiguration();
  }
}
