package frc.robot.mutables;

import frc.robot.utils.TunableValues.TunableNum;
import lombok.Getter;
import lombok.Setter;

/**
 * A mutable class to represent the desired elevator pose of the robot.
 */
public final class MutablePivotPosition {
  private static TunableNum tunablePivotPosition = new TunableNum("Pivot/TuneablePosition", 0.0);
  
  /**
   * An enum to represent all desired elevator positions.
   */
  public static enum PivotPosition {
    INTAKE_READY(0.0),
    ELEVATOR_CLEAR(0.1),
    L1(0),
    L2(0.2),
    L3(0.4),
    L4(0.6),
    BARGE(0.8);

    private final double position;

    PivotPosition(double position) {
      this.position = position;
    }

    public double getAsDouble() {
      return position;
    }
  }

  @Getter
  @Setter
  private static PivotPosition mutablePivotPosition = PivotPosition.ELEVATOR_CLEAR;

  /**
   * Returns the desired pivot position as a double.
   *
   * @return the pivot elevator position as a double
   */
  public static double getMutablePivotPositionAsDouble() {
    return mutablePivotPosition.getAsDouble();
  }

  /**
   * Get the tunable pivot position. To be used in test mode to figure out
   * desired pivot positions.
   * 
   * @return the tunable pivot position
   */
  public static double getTunablePivotPositionAsDouble() {
    return tunablePivotPosition.getAsDouble();
  }
}
