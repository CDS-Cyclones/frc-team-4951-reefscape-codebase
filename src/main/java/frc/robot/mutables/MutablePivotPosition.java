package frc.robot.mutables;

import frc.robot.utils.TunableValues.TunableNum;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/**
 * A mutable class to represent the mutable pivot position of the robot.
 */
public final class MutablePivotPosition {
  private static TunableNum tunablePivotPosition = new TunableNum("Pivot/TuneablePosition", 0.0);
  
  /**
   * An enum to represent all mutable pivot positions.
   */
  @RequiredArgsConstructor
  public static enum PivotPosition {
    INTAKE_READY(0.0),
    ELEVATOR_CLEAR(0.1),
    L1(0.0),
    L2(0.2),
    L3(0.4),
    L4(0.6),
    BARGE(0.8),
    PROCESSOR(0.2),
    TUNABLE(Double.NaN); // Special value for tunable position

    private final double position;

    public double getAsDouble() {
      if (this == TUNABLE) {
        return tunablePivotPosition.getAsDouble();
      }
      return position;
    }
  }

  @Getter
  @Setter
  private static PivotPosition mutablePivotPosition = PivotPosition.ELEVATOR_CLEAR;
}