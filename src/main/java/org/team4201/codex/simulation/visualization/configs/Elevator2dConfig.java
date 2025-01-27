package org.team4201.codex.simulation.visualization.configs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.VisualizationUtils.ELEVATOR_TYPE;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

/**
 * Configuration used for an {@link Elevator2d}
 *
 */
public class Elevator2dConfig extends BaseMechanismConfig {
    /** Initial width (thickness) of the arm. Default is 2 inches if not set */
    public Distance m_initialWidth;

    /** Number of stages the elevator uses. Used for visualization */
    public int m_numberOfStages;

    /** Initial length of the elevator */
    public Distance m_initialLength;

    /** Max lengths of each stage (For CONTINUOUS elevators) */
    public Distance[] m_stageMaxLengths;

    /** Angle offset of the elevator to its parent {@link MechanismLigament2d} */
    public Angle m_angleOffset = Degrees.of(0);

    /** {@link Elevator2d} type (for visualization). Default is CASCADE */
    public ELEVATOR_TYPE m_type = ELEVATOR_TYPE.CASCADE;

    /**
     * Initialize a config for the {@link Elevator2d}.
     *
     * @param name Name of the mechanism
     */
    public Elevator2dConfig(String name) {
        this(name, new Color8Bit(255, 0, 0), Inches.of(24));
    }

    /**
     * Initialize a config for the {@link Elevator2d}.
     *
     * @param name Name of the mechanism
     * @param color {@link Color8Bit} to use for the mechanism
     * @param initialLength Initial length of the Elevator2d
     */
    public Elevator2dConfig(String name, Color8Bit color, Distance initialLength) {
        this(name, color, initialLength, 1);
    }

    /**
     * Initialize a config for the {@link Elevator2d}.
     *
     * @param name Name of the mechanism
     * @param color {@link Color8Bit} to use for the mechanism
     * @param initialLength Initial length of the Elevator2d
     * @param numberOfStages Number of stages in the Elevator2d. Default is 1
     */
    public Elevator2dConfig(String name, Color8Bit color, Distance initialLength, int numberOfStages) {
        this(name, color, initialLength, numberOfStages, Inches.of(2));

    }

    /**
     * Initialize a config for the {@link Elevator2d}.
     *
     * @param name Name of the mechanism
     * @param color {@link Color8Bit} to use for the mechanism
     * @param initialLength Initial length of the Elevator2d
     * @param numberOfStages Number of stages in the Elevator2d. Default is 1
     * @param initialWidth Initial width (thickness) of the Elevator. Defaults to 2 inches.
     */
    public Elevator2dConfig(String name, Color8Bit color, Distance initialLength, int numberOfStages, Distance initialWidth) {
        super(name, color);

        m_initialLength = initialLength;
        m_numberOfStages = numberOfStages;
        m_initialWidth = initialWidth;
    }

    /**
     * Set the angle offset of the {@link Elevator2d} to its parent mechanism
     *
     * @param angleOffset The {@link Angle} offset of the Elevator from its parent
     */
    public void setAngleOffset(Angle angleOffset){
        m_angleOffset = angleOffset;
    }

    /**
     *  Set the max length for each stage for the {@link Elevator2d} (For visualization).
     *  Must be set for CONTINUOUS elevators.
     *
     * @param stageMaxLengths The max {@link Distance} each stage can can reach.
     */
    public void setStageMaxLengths(Distance... stageMaxLengths) {
        m_stageMaxLengths = stageMaxLengths;
        m_numberOfStages = m_stageMaxLengths.length;
    }

    /**
     *  Set the {@link Elevator2d} type (For visualization)
     *
     * @param type The {@link ELEVATOR_TYPE}
     */
    public void setElevatorType(ELEVATOR_TYPE type) {
        m_type = type;
    }

    /**
     * Copy the {@link Elevator2dConfig} object.
     *
     * @return {@link Elevator2dConfig}
     */
    public Elevator2dConfig clone() {
        return new Elevator2dConfig(m_name, m_color, m_initialLength, m_numberOfStages, m_initialWidth);
    }
}
