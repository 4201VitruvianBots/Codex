package edu.wpi.first.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class TestGeometry {

  @Test
  public void testPose2d() {
    Pose2d origin = new Pose2d(3, 2, Rotation2d.kZero);
    Translation2d translation = new Translation2d(2, 0);
    Pose2d referencePoint = new Pose2d(3 + Math.sqrt(2), 2 + Math.sqrt(2), Rotation2d.kZero);

    // No rotation applied
    //        Pose2d newPoint = origin.plus(new Transform2d(translation,
    // Rotation2d.fromDegrees(45)));

    // ???
    //        Pose2d newPoint = origin.rotateBy(Rotation2d.fromDegrees(45)).plus(new
    // Transform2d(translation, Rotation2d.kZero));

    // Works, but feels like this should be simplified
    Pose2d newPoint =
        new Pose2d(origin.getTranslation(), Rotation2d.fromDegrees(45))
            .plus(new Transform2d(translation, Rotation2d.kZero));

    assertEquals(referencePoint.getTranslation(), newPoint.getTranslation());

    //        var transformA = origin.minus(newPoint);
    //        var transformB = newPoint.minus(origin);
    //
    //        Pose2d test2 = origin.plus(new Transform2d(translation.unaryMinus(),
    // Rotation2d.fromDegrees(-45)));
    //
    //        assertEquals(referencePoint.getTranslation(), test2.getTranslation());
  }
}
