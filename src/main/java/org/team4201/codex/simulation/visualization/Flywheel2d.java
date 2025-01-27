// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.codex.simulation.visualization;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.team4201.codex.simulation.visualization.configs.Flywheel2dConfig;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 *  Class to represent a flywheel using {@link Mechanism2d}
 *  */
public class Flywheel2d implements AutoCloseable {
  private int NUM_SIDES = 8;

  private final Flywheel2dConfig m_config;
  private final MechanismLigament2d[] m_sides = new MechanismLigament2d[NUM_SIDES];
  private final MechanismLigament2d m_flywheel;
  private MechanismObject2d m_parentObject;
  private Flywheel2d m_subFlywheel2d;

  /**
   * Create a new {@link Flywheel2d} instance
   *
   * @param config The {@link Flywheel2dConfig} that defines the Flywheel2d parameters
   */
  public Flywheel2d(Flywheel2dConfig config) {
    this(config, null);
  }

  /**
   * Create a new {@link Flywheel2d} instance
   *
   * @param config The {@link Flywheel2dConfig} that defines the Flywheel2d parameters
   * @param parentObject The {@link MechanismObject2d} (Either a {@link MechanismRoot2d} or
   *                     {@link MechanismLigament2d})the Flywheel attaches to
   */
  public Flywheel2d(Flywheel2dConfig config, MechanismLigament2d parentObject) {
    m_config = config;
    m_flywheel = new MechanismLigament2d(
            m_config.m_name, m_config.m_initialRadius.in(Inches), 0, 3, m_config.m_color);
    initSides();

    if(parentObject != null) {
      m_parentObject = parentObject;

      Flywheel2dConfig flywheelSubConfig = m_config.clone();
      flywheelSubConfig.m_name = flywheelSubConfig.m_name + "_sub";
      m_subFlywheel2d = new Flywheel2d(flywheelSubConfig);
      m_parentObject.append(m_flywheel);
    }
  }

  private void initSides() {
    for(int i = 0; i < NUM_SIDES; i++) {
      if (i == 0) {
        m_sides[i] =
                new MechanismLigament2d(
                        m_config.m_name + "_side" + i, m_config.m_initialRadius.in(Inches), 112.5, 3, m_config.m_color);
        m_flywheel.append(m_sides[i]);
      } else {
        m_sides[i] =
                new MechanismLigament2d(
                        m_config.m_name + "_side" + i, m_config.m_initialRadius.in(Inches), 45, 3, m_config.m_color);
        m_sides[i-1].append(m_sides[i]);
      }
    }
  }

  /**
   *  Get the {@link Flywheel2d}'s {@link Flywheel2dConfig}
   *
   * @return {@link Flywheel2dConfig}
   */
  public Flywheel2dConfig getConfig() {
    return m_config;
  }

  /**
   * Get the {@link Flywheel2d}'s base {@link MechanismLigament2d}
   *
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getLigament() {
    return m_flywheel;
  }

  /**
   * Get the {@link Flywheel2d}'s parent {@link MechanismObject2d}. Returns null if it doesn't exist.
   *
   * @return {@link MechanismObject2d}
   */
  public MechanismObject2d getParentObject() {
    return m_parentObject;
  }

  /**
   * Get the {@link Flywheel2d}'s subMechanism. For displaying the mechanism by itself.
   *
   * @return {@link Flywheel2d}
   */
  public Flywheel2d getSubFlywheel() {
    return m_subFlywheel2d;
  }

  /**
   * Update the {@link Flywheel2d}'s position for visualization.
   *
   * @param rps The velocity of the {@link Flywheel2d}. Used to change the color of the {@link Flywheel2d} for visualization.
   */
  public void update(AngularVelocity rps) {
    m_flywheel.setAngle(m_flywheel.getAngle() - 360 * rps.in(RotationsPerSecond) * 0.2);

    if(m_subFlywheel2d != null) {
      m_subFlywheel2d.update(rps);
    }
  }

  @Override
  public void close() throws Exception {
    m_flywheel.close();
    for (var side: m_sides) {
      side.close();
    }

    if(m_subFlywheel2d != null) {
      m_subFlywheel2d.close();
    }
  }
}
