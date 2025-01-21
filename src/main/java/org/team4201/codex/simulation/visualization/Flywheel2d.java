// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.codex.simulation.visualization;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 *  Class to represent a flywheel using {@link Mechanism2d}
 *  */
public class Flywheel2d {
  private int NUM_SIDES = 8;
  private final MechanismLigament2d m_flywheel;
  private final MechanismLigament2d m_spinner;
  private final MechanismLigament2d[] m_sides = new MechanismLigament2d[NUM_SIDES];

  private final String m_mechanismName;
  private final double m_flywheelRadius;


  /**
   * Create a new {@link Flywheel2d} instance
   *
   * @param name The name of the object (Must be unique across all {@link Mechanism2d} objects)
   * @param flywheelRadius The radius of the flywheel
   */
  public Flywheel2d(String name, double flywheelRadius) {
    m_mechanismName = name;
    m_flywheelRadius = flywheelRadius;

    m_flywheel = new MechanismLigament2d(name, 1, 0);

    m_spinner =
            m_flywheel.append(
                    new MechanismLigament2d(
                            m_mechanismName, Units.inchesToMeters(1.7), 0, 0, new Color8Bit(Color.kAliceBlue)));

    initSides();
  }

  private void initSides() {
    for(int i = 0; i < NUM_SIDES - 1; i++) {
      if (i == 0) {
        m_spinner.append(
                new MechanismLigament2d(
                        m_mechanismName + "_side" + i, m_flywheelRadius, 112.5, 3, new Color8Bit(Color.kDimGray)));
      } else {
        m_sides[i+1] =
                m_sides[i].append(
                        new MechanismLigament2d(
                                m_mechanismName + "_side" + i, m_flywheelRadius, 45, 3, new Color8Bit(Color.kDimGray)));
      }
    }
  }

  /**
   * Get the {@link Flywheel2d}'s base {@link MechanismLigament2d}
   *
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getLigament() {
    return m_spinner;
  }

  /**
   * Update the {@link Flywheel2d}'s position for visualization.
   *
   * @param rpm The velocity of the {@link Flywheel2d}. Used to change the color of the {@link Flywheel2d} for visualization.
   */
  public void update(double rpm) {
    m_spinner.setAngle(m_spinner.getAngle() - 360 * rpm / 60 * 0.2);
  }
}
