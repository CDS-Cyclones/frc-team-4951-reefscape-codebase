package frc.robot.mutables;

import frc.robot.utils.TunableValues.TunableNum;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/**
 * A mutable class to represent the mutable elevator position of the robot.
 */
public final class MutableElevatorPosition {
  private static TunableNum tunableElevatorPosition = new TunableNum("Elevator/TuneablePosition", 0.0);

  /**
   * An enum to represent all mutable elevator positions.
   */
  @RequiredArgsConstructor
  public static enum ElevatorPosition {
    DOWN(0.0),
    L1(0.1),
    L2(0.2),
    L3(0.3),
    L4(0.4),
    BARGE(0.5),
    PROCESSOR(0.6),
    TUNABLE(Double.NaN); // Special value for tunable position

    private final double position;

    public double getAsDouble() {
      if (this == TUNABLE) {
        return tunableElevatorPosition.getAsDouble();
      }
      return position;
    }
  }

  @Getter
  @Setter
  private static ElevatorPosition mutableElevatorPosition = ElevatorPosition.DOWN;
}