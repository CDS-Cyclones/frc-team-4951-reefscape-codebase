package frc.robot.mutables;

import frc.robot.utils.TunableValues.TunableNum;
import lombok.Getter;
import lombok.Setter;

/**
 * A mutable class to represent the desired elevator pose of the robot.
 */
public final class MutableElevatorPosition {
  private static TunableNum tunableElevatorPosition = new TunableNum("Elevator/TuneablePosition", 0.0);

  /**
   * An enum to represent all desired elevator positions.
   */
  public static enum ElevatorPosition {
    DOWN(0.0),
    L1(0.1),
    L2(0.2),
    L3(0.3),
    L4(0.4),
    BARGE(0.5);

    private final double position;

    ElevatorPosition(double position) {
      this.position = position;
    }

    public double getAsDouble() {
      return position;
    }
  }

  @Getter
  @Setter
  private static ElevatorPosition mutableElevatorPosition = ElevatorPosition.DOWN;

  /**
   * Returns the desired elevator position as a double.
   *
   * @return the desired elevator position as a double
   */
  public static double getMutableElevatorPositionAsDouble() {
    return mutableElevatorPosition.getAsDouble();
  }

  /**
   * Get the tunable elevator position. To be used in test mode to figure out
   * desired elevator positions.
   * 
   * @return the tunable pivot position
   */
  public static double getTunableElevatorPositionAsDouble() {
    return tunableElevatorPosition.getAsDouble();
  }
}
