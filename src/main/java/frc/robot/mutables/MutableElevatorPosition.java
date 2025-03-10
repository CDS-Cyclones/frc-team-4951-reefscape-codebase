package frc.robot.mutables;

import lombok.Getter;
import lombok.Setter;

/**
 * A mutable class to represent the desired elevator pose of the robot.
 */
public final class MutableElevatorPosition {
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

    public double getPosition() {
      return position;
    }
  }

  @Getter
  @Setter
  private static ElevatorPosition mutableElevatorPositions = ElevatorPosition.DOWN;

  /**
   * Returns the desired elevator position.
   *
   * @return The desired elevator position.
   */
  public static double getDesiredElevatorPosition() {
    return mutableElevatorPositions.getPosition();
  }
}
