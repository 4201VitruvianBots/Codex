package org.team4201.codex.simulation.visualization;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 *  Class to represent an elevator using {@link Mechanism2d}
 *  */
public class Elevator2d implements AutoCloseable {
  private final MechanismLigament2d[] m_elevator2d= new MechanismLigament2d[5];
  private final Color8Bit m_ligamentColor = new Color8Bit(52, 212, 235);
  private final String m_name;

  /**
   * Create a new {@link Elevator2d} instance
   *
   * @param name The name of the object (Must be unique across all {@link Mechanism2d} objects)
   */
  public Elevator2d(String name) {
    m_name = name;

    // Create lines to represent the climber post
    m_elevator2d[0] =
        new MechanismLigament2d(m_name + "Post", 1, 90, 25, m_ligamentColor);
  }

  /**
   * Get the {@link Elevator2d}'s base {@link MechanismLigament2d}
   *
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getLigament() {
    return m_elevator2d[0];
  }

  /**
   * Update the {@link Elevator2d}'s position relative to its attachment point
   *
   * @param height The height to set the {@link Elevator2d} to.
   */
  public void update(double height) {
    update(height, 0);
  }

  /**
   * Update the {@link Elevator2d}'s position relative to its attachment point
   *
   * @param height The height to set the {@link Elevator2d} to.
   * @param velocity The velocity of the {@link Elevator2d}. Used to change the color of the {@link Elevator2d} for visualization.
   */
  public void update(double height, double velocity) {
//    m_climber2d.setLength(height + CLIMBER.kHookHeight);

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizationUtils.updateMotorColor(m_elevator2d[0], velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    for (var elevatorSegment: m_elevator2d) {
      elevatorSegment.close();
    }
  }
}
