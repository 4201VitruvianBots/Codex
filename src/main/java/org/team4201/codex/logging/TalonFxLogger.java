package org.team4201.codex.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import org.team4201.codex.robot.hardware.LoggedTalonFX;

@CustomLoggerFor(LoggedTalonFX.class)
public class TalonFxLogger extends ClassSpecificLogger<LoggedTalonFX> {

  public TalonFxLogger() {
    super(LoggedTalonFX.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, LoggedTalonFX loggedTalonFX) {
    for (var entry : loggedTalonFX.getLoggedSignals().values()) {
      dataLogger.log(entry.getName(), entry.getValueAsDouble());
    }
  }
}
