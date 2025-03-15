package frc.robot.mutables;

public class MutableIntakeState {
  public enum IntakeState {
    EMPTY,
    CORAL,
    ALGA
  }
    
  private static IntakeState mutableIntakeState = IntakeState.EMPTY;
    
  public static IntakeState getMutableIntakeState() {
    return mutableIntakeState;
  }
    
  public static void setMutableIntakeState(IntakeState state) {
    mutableIntakeState = state;
  }
}
