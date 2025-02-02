package org.team4201.codex.simulation.visualization.configs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.team4201.codex.simulation.visualization.Arm2d;

/** Configuration used for an {@link Arm2d} */
public class Arm2dConfig extends BaseMechanismConfig {
  /** Initial width (thickness) of the arm. Default is 2 inches if not set */
  public Distance m_initialWidth;

  /** Initial length of the arm */
  public Distance m_initialLength;

  /** Initial angle of the arm */
  public Angle m_initialAngle;

  /** Angle offset of the arm to its parent {@link MechanismLigament2d} */
  public Angle m_angleOffset = Degrees.of(0);

  /**
   * Initialize a config for the {@link Arm2d}.
   *
   * @param name Name of the mechanism
   */
  public Arm2dConfig(String name) {
    this(name, new Color8Bit(255, 255, 255), Degrees.of(0), Inches.of(12));
  }

  /**
   * Initialize a config for the {@link Arm2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialAngle Initial angle the Arm2d is at (0 degrees is parallel to ground)
   * @param initialLength Initial length of the Arm2d
   */
  public Arm2dConfig(String name, Color8Bit color, Angle initialAngle, Distance initialLength) {
    this(name, color, initialAngle, initialLength, Inches.of(2));
  }

  /**
   * Initialize a config for the {@link Arm2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialAngle Initial angle the Arm2d is at (0 degrees is parallel to ground)
   * @param initialLength Initial length of the Arm2d
   * @param initialWidth Initial width (thickness) of the Arm2d. Defaults to 2 inches.
   */
  public Arm2dConfig(
      String name,
      Color8Bit color,
      Angle initialAngle,
      Distance initialLength,
      Distance initialWidth) {
    super(name, color);

    m_initialAngle = initialAngle;
    m_initialLength = initialLength;
    m_initialWidth = initialWidth;
  }

  /**
   * Set the angle offset of the {@link Arm2d} to its parent mechanism
   *
   * @param angleOffset The {@link Angle} offset of the Arm2d from its parent
   */
  public void setAngleOffset(Angle angleOffset) {
    m_angleOffset = angleOffset;
  }

  /**
   * Copy the {@link Arm2dConfig} object.
   *
   * @return {@link Arm2dConfig}
   */
  public Arm2dConfig clone() {
    return new Arm2dConfig(m_name, m_color, m_initialAngle, m_initialLength, m_initialWidth);
  }
}
