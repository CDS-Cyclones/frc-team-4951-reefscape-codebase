package frc.robot;

import lombok.Getter;
import lombok.Setter;

import java.util.function.DoubleSupplier;

/**
 * A class to store all the positions of the elevator and pivot.
 */
public final class ManipulatorSubsystemsPositions {
  @Getter
  @Setter
  public static class MutableElevatorPosition {
    private double position;

    public MutableElevatorPosition(double position) {
      this.position = position;
    }

    public DoubleSupplier getSupplier() {
      return () -> position;
    }
  }

  public enum ElevatorPosition {
    DOWN(new MutableElevatorPosition(0.0)),
    L1(new MutableElevatorPosition(0.0)),
    L2(new MutableElevatorPosition(0.5)),
    L3(new MutableElevatorPosition(1.0)),
    L4(new MutableElevatorPosition(1.5)),
    BARGE(new MutableElevatorPosition(2.0));

    private final MutableElevatorPosition position;

    ElevatorPosition(MutableElevatorPosition position) {
      this.position = position;
    }

    public double getPosition() {
      return position.getPosition();
    }

    public void setPosition(double position) {
      this.position.setPosition(position);
    }

    public DoubleSupplier getSupplier() {
      return position.getSupplier();
    }
  }

  @Getter
  @Setter
  public static class MutablePivotPosition {
    private double position;

    public MutablePivotPosition(double position) {
      this.position = position;
    }

    public DoubleSupplier getSupplier() {
      return () -> position;
    }
  }

  public enum PivotPosition {
    IN(new MutablePivotPosition(0.0)),
    OUT(new MutablePivotPosition(1.0));

    private final MutablePivotPosition position;

    PivotPosition(MutablePivotPosition position) {
      this.position = position;
    }

    public double getPosition() {
      return position.getPosition();
    }

    public void setPosition(double position) {
      this.position.setPosition(position);
    }

    public DoubleSupplier getSupplier() {
      return position.getSupplier();
    }
  }

  @Setter
  private static ElevatorPosition currentElevatorPosition = ElevatorPosition.DOWN;
  @Setter
  private static PivotPosition currentPivotPosition = PivotPosition.IN;

  public static DoubleSupplier getCurrentElevatorPositionSupplier() {
    return currentElevatorPosition.getSupplier();
  }

  public static DoubleSupplier getCurrentPivotPositionSupplier() {
    return currentPivotPosition.getSupplier();
  }

}
