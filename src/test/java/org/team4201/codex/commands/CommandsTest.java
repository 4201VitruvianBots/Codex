package org.team4201.codex.commands;

import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

@Disabled
public class CommandsTest extends CommandTestBase {
  private boolean m_trigger;

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test. We need to close each subsystem properly, as they all get
  // separately initialized for each individual test, which will trip some errors due to how WPILib
  // is set up (e.g. resource errors from using the same PWM/DIO port)
  void shutdown() throws Exception {}

  // Mark all test functions with @Test
  @Test
  public void TestInterruptingCommand() {
    var mockCommand1 = new MockCommandHolder(true);
    var mockCommand2 = new MockCommandHolder(true);
    var command1 = mockCommand1.getMock();
    var command2 = mockCommand2.getMock();

    BooleanSupplier trigger = this::getTrigger;

    var interruptingCommand = new InterruptingCommand(command1, command2, trigger);

    CommandScheduler.getInstance().schedule(interruptingCommand);
    CommandScheduler.getInstance().run();

    verify(command1).initialize();
    verify(command1).execute();

    verify(command2, never()).initialize();
    verify(command2, never()).execute();
    verify(command2, never()).end(false);

    setTrigger(true);
    CommandScheduler.getInstance().run();

    verify(command1).end(true);

    verify(command2).initialize();
    verify(command2).execute();
  }

  @Test
  public void TestDelayedInterruptingCommand() {
    var mockCommand1 = new MockCommandHolder(true);
    var mockCommand2 = new MockCommandHolder(true);
    var command1 = mockCommand1.getMock();
    var command2 = mockCommand2.getMock();

    BooleanSupplier trigger = this::getTrigger;

    var interruptingCommand = new DelayedInterruptingCommand(command1, command2, 0.01, trigger);

    CommandScheduler.getInstance().schedule(interruptingCommand);
    CommandScheduler.getInstance().run();

    verify(command1).initialize();
    verify(command1).execute();

    verify(command2, never()).initialize();
    verify(command2, never()).execute();
    verify(command2, never()).end(false);

    setTrigger(true);
    CommandScheduler.getInstance().run();

    verify(command2, never()).initialize();
    verify(command2, never()).execute();
    verify(command2, never()).end(false);

    SimHooks.stepTiming(1);
    CommandScheduler.getInstance().run();

    verify(command1).end(true);

    verify(command2).initialize();
    verify(command2).execute();
  }

  private boolean getTrigger() {
    return m_trigger;
  }

  private void setTrigger(boolean state) {
    m_trigger = state;
  }
}
