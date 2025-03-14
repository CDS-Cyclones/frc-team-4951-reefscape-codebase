package frc.robot.mutables;


import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/**
 * A mutable class to represent the desired state of the candle.
 */
public class MutableCandleState {
  @RequiredArgsConstructor
  public enum CandleState {
    OFF(0, 0, 0),
    TARGET_FOUND(255, 188, 0), // orange
    AT_POSE(128, 255, 0);  // green

    @Getter private final int red;
    @Getter private final int green;
    @Getter private final int blue;

    @Override
    public String toString() {
      return "RGB: (" + red + ", " + green + ", " + blue + ")";
    }
  }

  @Getter @Setter
  private static CandleState mutableCandleState = CandleState.OFF;
}