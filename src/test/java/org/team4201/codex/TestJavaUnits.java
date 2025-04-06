package org.team4201.codex;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import org.junit.jupiter.api.Test;

public class TestJavaUnits {
  @Test
  public void testMutableValues() {

    var mutableMeasure = Feet.mutable(0);
    var measure = mutableMeasure.in(Inches);
    System.out.println("Measure Value:" + measure);

    mutableMeasure.mut_replace(Feet.of(1));
    System.out.println("Measure Value:" + measure);
  }
}
