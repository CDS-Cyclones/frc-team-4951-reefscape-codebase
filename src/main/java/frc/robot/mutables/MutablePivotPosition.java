package frc.robot.mutables;

import lombok.Getter;
import lombok.Setter;

/**
 * A mutable class to represent the desired elevator pose of the robot.
 */
public final class MutablePivotPosition {
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

    public double getPosition() {
      return position;
    }
  }

  @Getter
  @Setter
  private static PivotPosition mutablePivotPosition = PivotPosition.ELEVATOR_CLEAR;
}
