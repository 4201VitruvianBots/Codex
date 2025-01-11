package org.team4201.codex.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Class to handle all updates to the Field2D widget
 */
public class FieldSim implements AutoCloseable, Subsystem {
    private final Field2d m_field2D = new Field2d();

    private Map<String, Pose2d[]> m_objectPoses = new HashMap<>();

    private String[] m_protectedKeys = {
            "robotPose",
            "modulePoses"
    };

    /**
     * Create a FieldSim object
     */
    public FieldSim() {
        SmartDashboard.putData("Field2D", m_field2D);
    }

    /**
     * Add a pose to FieldSim to display
     *
     * @param key The name of the object (Must be unique)
     * @param poses The poses corresponding to the object's position
     */
    public void addPoses(String key, Pose2d... poses) {
        m_objectPoses.put(key, poses);
    }

    /**
     * Remove a pose from being displayed on FieldSim
     *
     * @param key The name of the object to remove
     */
    public void removePoses(String key) {
        m_objectPoses.remove(key);
    }

    /**
     * Remove all poses from being displayed on FieldSim
     */
    public void clearAllPoses() {
        m_objectPoses.clear();
    }

    /**
     * Add a trajectory to be displayed in the Field2D widget.
     *
     * @param trajectory The wpilib Trajectory to display
     */
    public void addTrajectory(Trajectory trajectory) {
        m_field2D.getObject("path").setTrajectory(trajectory);
    }

    private void updateField2d() {
        m_field2D.setRobotPose(m_objectPoses.get("robotPose")[0]);

        for (var entry:m_objectPoses.entrySet()) {
            if (Arrays.asList(m_protectedKeys).contains(entry.getKey())) {
                continue;
            }
            m_field2D.getObject(entry.getKey()).setPoses(entry.getValue());
        }

        if (RobotBase.isSimulation()) {
            m_field2D.getObject("Swerve Modules").setPoses(m_objectPoses.get("modulePoses"));
        }
    }

    @Override
    public void periodic() {
        updateField2d();
    }

    @Override
    public void simulationPeriodic() {}

    @SuppressWarnings("RedundantThrows")
    @Override
    public void close() throws Exception {}
}
