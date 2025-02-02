package org.team4201.codex.simulation.visualization;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;

/** Class to represent an elevator using {@link Mechanism2d} */
public class Elevator2d implements AutoCloseable {
  private final Elevator2dConfig m_config;
  private final MechanismLigament2d[] m_elevatorStages;
  private MechanismObject2d m_parentObject;
  private Elevator2d m_subElevator2d;

  /**
   * Create a new {@link Elevator2d} instance
   *
   * @param config The {@link Elevator2dConfig} that defines the Elevator2d parameters
   */
  public Elevator2d(Elevator2dConfig config) {
    this(config, null);
  }

  /**
   * Create a new {@link Elevator2d} instance
   *
   * @param config The {@link Elevator2dConfig} that defines the Elevator2d parameters
   * @param parentObject The {@link MechanismObject2d} (Either a {@link MechanismRoot2d} or {@link
   *     MechanismLigament2d})the Elevator2d attaches to
   */
  public Elevator2d(Elevator2dConfig config, MechanismObject2d parentObject) {
    m_config = config;

    m_elevatorStages = new MechanismLigament2d[m_config.m_numberOfStages];
    for (int i = 0; i < m_elevatorStages.length; i++) {
      m_elevatorStages[i] =
          new MechanismLigament2d(
              m_config.m_name + "_" + i,
              m_config.m_initialLength.in(Inches),
              m_config.m_angleOffset.in(Degrees),
              m_config.m_initialWidth.in(Inches),
              m_config.m_color);
      if (i > 0) {
        m_elevatorStages[i - 1].append(m_elevatorStages[i]);
      }
    }

    if (parentObject != null) {
      m_parentObject = parentObject;

      Elevator2dConfig elevatorSubConfig = m_config.clone();
      elevatorSubConfig.m_name = elevatorSubConfig.m_name + "_sub";
      m_subElevator2d = new Elevator2d(elevatorSubConfig);
      m_parentObject.append(getLigament());
    }
  }

  /**
   * Get the {@link Elevator2d}'s {@link Elevator2dConfig}
   *
   * @return {@link Elevator2dConfig}
   */
  public Elevator2dConfig getConfig() {
    return m_config;
  }

  /**
   * Get the {@link Elevator2d}'s base {@link MechanismLigament2d} stage.
   *
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getLigament() {
    return m_elevatorStages[0];
  }

  /**
   * Get one of the {@link Elevator2d}'s {@link MechanismLigament2d} stage.
   *
   * @param index The stage index to return
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getLigament(int index) {
    return m_elevatorStages[index];
  }

  /**
   * Get the {@link Elevator2d}'s subMechanism. For displaying the mechanism by itself.
   *
   * @return {@link Elevator2d}
   */
  public Elevator2d getSubElevator() {
    return m_subElevator2d;
  }

  /**
   * Set the {@link Elevator2d}'s {@link Angle}.
   *
   * @param angle Angle of the Elevator2d
   */
  public void setAngle(Angle angle) {
    setAngle(angle, angle);
  }

  /**
   * Set the {@link Elevator2d}'s {@link Angle}. This function is used if you want a separate
   * orientation for the standalone display of the Elevator2d.
   *
   * @param angle Angle of the Elevator2d
   * @param subAngle Angle of the Elevator2d's subMechanism.
   */
  public void setAngle(Angle angle, Angle subAngle) {
    getLigament().setAngle(angle.in(Degrees));

    if (m_subElevator2d != null) {
      m_subElevator2d.getLigament().setAngle(subAngle.in(Degrees));
    }
  }

  /**
   * Update the {@link Elevator2d}'s position relative to its attachment point
   *
   * @param height The height to set the {@link Elevator2d} to.
   */
  public void update(Distance height) {
    update(height, FeetPerSecond.of(0));
  }

  /**
   * Update the {@link Elevator2d}'s position relative to its attachment point
   *
   * @param height The height to set the {@link Elevator2d} to.
   * @param velocity The velocity of the {@link Elevator2d}. Used to change the color of the {@link
   *     Elevator2d} for visualization.
   */
  public void update(Distance height, LinearVelocity velocity) {
    switch (m_config.m_type) {
      case CONTINUOUS:
        var subHeight = height.copy();
        for (int i = 0; i < m_config.m_numberOfStages; i++) {
          if (m_config.m_stageMaxLengths[i].lte(subHeight)) {
            m_elevatorStages[i].setLength(m_config.m_stageMaxLengths[i].in(Inches));
            subHeight.minus(m_config.m_stageMaxLengths[i]);
          } else {
            m_elevatorStages[i].setLength(subHeight.in(Inches));
            break;
          }
        }
        break;
      case CASCADE:
      default:
        for (var stage : m_elevatorStages) {
          stage.setLength(height.in(Inches) / m_config.m_numberOfStages);
        }
    }

    for (var stage : m_elevatorStages) {
      VisualizationUtils.updateMotorColor(stage, velocity.in(FeetPerSecond), m_config.m_color);
    }

    if (m_subElevator2d != null) {
      m_subElevator2d.update(height, velocity);
    }
  }

  @Override
  public void close() throws Exception {
    for (var elevatorSegment : m_elevatorStages) {
      elevatorSegment.close();
    }

    if (m_subElevator2d != null) {
      m_subElevator2d.close();
    }
  }
}
