package frc.robot;

/**
 * A class to store all the positions of the elevator and pivot.
 */
public final class Positions {
  public
  enum ElevatorPosition {
    DOWN(0.0),
    L1(0.0),
    L2(0.5),
    L3(1.0),
    L4(1.5),
    BARGE(2.0);

    private final double position;

    ElevatorPosition(double position){this.position = position;}

    public double getPosition() { return position; }
  }

  public enum PivotPosition {
    IN(0.0),
    OUT(1.0);

    private final double position;

    PivotPosition(double position){this.position = position;}

    public double getPosition() { return position; }
  }
}
