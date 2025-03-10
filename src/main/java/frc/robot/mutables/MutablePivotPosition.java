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
    FULL_IN(0.0),      // To be able to intake through funnel
    REST_IN(0.1),      // Doesn't stick, but isnt in elevator's way
    SCORE_CORAL(0.5),  // To be able to score coral   
    SCORE_ALGA(0.6);   // To be able to score alga

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
  private static PivotPosition mutableElevatorPositions = PivotPosition.FULL_IN;

  /**
   * Returns the desired pivot position.
   *
   * @return The desired pivot position.
   */
  public static double getDesiredPivotPosition() {
    return mutableElevatorPositions.getPosition();
  }
}
