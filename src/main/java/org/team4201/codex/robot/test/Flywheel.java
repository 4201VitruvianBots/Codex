package org.team4201.codex.robot.test;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import org.team4201.codex.robot.subsystems.RotationalTalonFxSubsystem;
import org.team4201.codex.robot.subsystems.RotationalTalonFxSubsystemInterface;
import org.team4201.codex.utils.CtreUtils;

public class Flywheel extends RotationalTalonFxSubsystem<Flywheel.Config> {

  public <ConfigT extends Flywheel.Config> Flywheel(ConfigT config) {
    super(config);
  }

  @Override
  public void userPeriodic() {
  }

  public static class Config extends RotationalTalonFxSubsystemInterface.Config {
    private Config() {
      super();
      this.gearRatio = 2.5 / 1.0;
      // TODO: Fix pro check in unit test/simulation
      //      this.gearbox = DCMotor.getKrakenX60Foc(2);
      this.gearbox = DCMotor.getKrakenX60(2);

      CANcoder encoder = new CANcoder(0);
      TalonFX motorA = new TalonFX(0);
      TalonFX motorB = new TalonFX(1);
      TalonFXConfiguration mainConfig = new TalonFXConfiguration();
      mainConfig.Slot0.kP = 10.0;
      mainConfig.Slot0.kI = 0;
      mainConfig.Slot0.kD = 0;
      mainConfig.Feedback.SensorToMechanismRatio = this.gearRatio;
      mainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      mainConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      mainConfig.Feedback.withRemoteCANcoder(encoder);
      //        mainConfig.Feedback.withFusedCANcoder(encoder);
      CtreUtils.configureTalonFx(motorA, mainConfig);

      TalonFXConfiguration followerConfig = new TalonFXConfiguration();
      CtreUtils.configureTalonFx(motorB, followerConfig);
      motorB.setControl(new Follower(motorA.getDeviceID(), false));

      withMotors(motorA, motorB);
    }

    public static Config getPrimaryConfig() {
      return new Config();
    }

    public static Config getSecondaryConfig() {
      var secondaryConfig = new Config();
      return secondaryConfig;
    }
  }
}
