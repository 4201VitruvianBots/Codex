package org.team4201.codex.simulation.visualization;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 *  Class to represent an arm using {@link Mechanism2d}
 *  */
public class Arm2d implements AutoCloseable {
  private final MechanismLigament2d m_arm2d;
  private final Color8Bit m_ligamentColor = new Color8Bit(235, 137, 52);
  private final String m_name;

  /**
   * Create a new {@link Arm2d} instance
   *
   * @param name The name of the object (Must be unique across all {@link Mechanism2d} objects)
   */
  public Arm2d(String name) {
    m_name = name;

    // Create a "line" to represent the arm.
    // We will use this to show its current position
    m_arm2d = new MechanismLigament2d(m_name, 0, 0);
  }

  /**
   * Get the {@link Arm2d}'s {@link MechanismLigament2d}
   *
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getArmLigament() {
    return m_arm2d;
  }

  /**
   * Update the {@link Arm2d}'s position relative to its attachment point
   *
   * @param angle The angle to set the {@link Arm2d} to.
   */
  public void update(double angle) {
    update(angle, 0);
  }

  /**
   * Update the {@link Arm2d}'s position relative to its attachment point
   *
   * @param angle The angle to set the {@link Arm2d} to.
   * @param velocity The velocity of the {@link Arm2d}. Used to change the color of the {@link Arm2d} for visualization.
   */
  public void update(double angle, double velocity) {
//    m_arm2d.setAngle(307.5 - m_armJoint2d.getAngle() - angle);

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizationUtils.updateMotorColor(m_arm2d, velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    m_arm2d.close();
  }
}
