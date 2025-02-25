package org.team4201.codex.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** Class that implements {@link SwerveSubsystem} for simulation testing */
public class MockSwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements SwerveSubsystem {

  /** SwerveDrive max speed */
  public static final LinearVelocity maxSpeed = MetersPerSecond.of(5.96);

  /** SwerveDrive max angular speed */
  public static final AngularVelocity maxAngularRate = RotationsPerSecond.of(Math.PI * 0.3);

  private static final Distance kWheelRadius = Inches.of(2);

  PIDConstants autoTranslationConstants = new PIDConstants(10, 0, 0);
  PIDConstants autoRotationConstants = new PIDConstants(7, 0, 0);

  private static final Translation2d[] modulePositions = {
    new Translation2d(0.5, 0.5),
    new Translation2d(0.5, -0.5),
    new Translation2d(-0.5, 0.5),
    new Translation2d(-0.5, -0.5)
  };

  RobotConfig robotConfig =
      new RobotConfig(
          Pounds.of(125).in(Kilograms),
          6.883,
          new ModuleConfig(
              kWheelRadius.in(Meters),
              maxSpeed.in(MetersPerSecond),
              1.2,
              DCMotor.getKrakenX60(1),
              79,
              1),
          modulePositions);

  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(0)
          .withPigeon2Configs(null);

  private static final double kCoupleRatio = 3.125;

  private static final double kDriveGearRatio = 6.12;
  private static final double kSteerGearRatio = 21.42857142857143;

  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(0)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);

  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withSlipCurrent(Amps.of(120.0))
              .withSpeedAt12Volts(MetersPerSecond.of(5.96))
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withDriveMotorInitialConfigs(new TalonFXConfiguration())
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(new CANcoderConfiguration())
              .withSteerInertia(KilogramSquareMeters.of(0.01))
              .withDriveInertia(KilogramSquareMeters.of(0.01))
              .withSteerFrictionVoltage(Volts.of(0.2))
              .withDriveFrictionVoltage(Volts.of(0.2));

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      moduleConstants =
          new SwerveModuleConstants[] {
            ConstantCreator.createModuleConstants(
                21,
                20,
                10,
                Rotations.of(0),
                Meters.of(modulePositions[0].getX()),
                Meters.of(modulePositions[0].getY()),
                false,
                false,
                false),
            ConstantCreator.createModuleConstants(
                23,
                22,
                11,
                Rotations.of(0),
                Meters.of(modulePositions[1].getX()),
                Meters.of(modulePositions[1].getY()),
                false,
                false,
                false),
            ConstantCreator.createModuleConstants(
                25,
                24,
                13,
                Rotations.of(0),
                Meters.of(modulePositions[2].getX()),
                Meters.of(modulePositions[2].getY()),
                false,
                false,
                false),
            ConstantCreator.createModuleConstants(
                27,
                26,
                13,
                Rotations.of(0),
                Meters.of(modulePositions[3].getX()),
                Meters.of(modulePositions[3].getY()),
                false,
                false,
                false),
          };

  /** {@link MockSwerveSubsystem} constructor */
  public MockSwerveSubsystem() {
    this(DrivetrainConstants, moduleConstants);
  }

  /**
   * {@link MockSwerveSubsystem} constructor
   *
   * @param drivetrainConstants {@link SwerveDrivetrainConstants}
   * @param modules {@link SwerveModuleConstants}
   */
  public MockSwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);

    if (Utils.isSimulation()) {
      this.startSimThread();
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public RobotConfig getAutoRobotConfig() {
    return robotConfig;
  }

  @Override
  public PIDConstants getAutoTranslationPIDConstants() {
    return autoTranslationConstants;
  }

  @Override
  public PIDConstants getAutoRotationPIDConstants() {
    return autoRotationConstants;
  }

  @Override
  public void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
    setControl(
        m_pathApplyRobotSpeeds
            .withSpeeds(chassisSpeeds)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
              .ifPresent(
                      allianceColor -> {
                        setOperatorPerspectiveForward(
                                allianceColor == DriverStation.Alliance.Red
                                        ? kRedAlliancePerspectiveRotation
                                        : kBlueAlliancePerspectiveRotation);
                        m_hasAppliedOperatorPerspective = true;
                      });
    }
  }
}
