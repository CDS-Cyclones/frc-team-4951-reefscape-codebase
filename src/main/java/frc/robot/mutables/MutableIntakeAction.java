package frc.robot.mutables;

import frc.robot.Constants.RobotStateConstants.IntakeState;
import frc.robot.utils.TunableValues.TunableNum;
import lombok.Getter;
import lombok.Setter;

/**
 * A mutable class to represent the desired intake actions of the robot.
 */
public final class MutableIntakeAction {
  private static TunableNum tunableIntakeSpeed = new TunableNum("Intake/TunableSpeed", 0.0);
  private static TunableNum tunableIntakeTime = new TunableNum("Intake/TunableTime", 0.0);

  /**
   * An enum to represent all desired intake actions.
   */
  public static enum IntakeAction {
    STOP(0.0),

    SCORE_L1_ENDLESS(0.2),  // TODO tune
    SCORE_L2_ENDLESS(0.2),  // TODO tune
    SCORE_L3_ENDLESS(0.2),  // TODO tune
    SCORE_L4_ENDLESS(0.2),  // TODO tune
    SCORE_BARGE_ENDLESS(1.0),  // TODO tune
    SCORE_PROCESSOR_ENDLESS(1.0),  // TODO tune
    INTAKE_REEF_ALGA_ENDLESS(-0.3), // TODO tune

    SCORE_L1_TIMED(0.2, 3.0),  // TODO tune
    SCORE_L2_TIMED(0.2, 3.0),  // TODO tune
    SCORE_L3_TIMED(0.2, 3.0),  // TODO tune
    SCORE_L4_TIMED(0.2, 3.0),  // TODO tune
    SCORE_BARGE_TIMED(1.0, 2.0),  // TODO tune
    SCORE_PROCESSOR_TIMED(1.0, 2.0),  // TODO tune
    INTAKE_REEF_ALGA_TIMED(-0.3, 1.5),  // TODO tune

    INTAKE_CORAL_CONDITIONAL(0.5, IntakeState.CORAL),  // TODO tune
    INTAKE_ALGA_CONDITIONAL(-0.5, IntakeState.ALGA),  // TODO tune

    TUNABLE(Double.NaN, Double.NaN); // Special values for tunable speed and duration

    private final double speed;
    private final Double time; // Nullable for endless and conditional
    private final IntakeState state; // Nullable for timed and endless

    // Constructor for endless actions
    IntakeAction(double speed) {
      this.speed = speed;
      this.time = null;
      this.state = null;
    }

    // Constructor for timed actions
    IntakeAction(double speed, double time) {
      this.speed = speed;
      this.time = time;
      this.state = null;
    }

    // Constructor for conditional actions
    IntakeAction(double speed, IntakeState state) {
      this.speed = speed;
      this.time = null;
      this.state = state;
    }

    /**
     * Returns the speed at which this action should run.
     * 
     * @return The speed at which this action should run.
     */
    public double getSpeed() {
      if (this == TUNABLE) {
        return tunableIntakeSpeed.getAsDouble();
      }
      return speed;
    }

    /**
     * Returns the time for which this action should run.
     * Only applicable for timed actions.
     * 
     * @return The time for which this action should run.
     */
    public Double getTime() {
      if (this == TUNABLE) {
        return tunableIntakeTime.getAsDouble();
      }
      return time;
    }

    /**
     * Retunrs the state in which intake should be in order for this action to end.
     * Only applicable for conditional actions.
     * 
     * @return The state in which intake should be in order for this action to end.
     */
    public IntakeState getEndState() {
      return state;
    }

    /**
     * Returns whether this action should run until interrupted.
     * 
     * @return Whether this action should run until interrupted.
     */
    public boolean isEndless() {
      return time == null && state == null;
    }

    /**
     * Returns whether this action should run for a certain amount of time.
     * 
     * @return Whether this action should run for a certain amount of time.
     */
    public boolean isTimed() {
      return time != null;
    }

    /**
     * Returns whether this action should run until a certain state is reached.
     * 
     * @return Whether this action should run until a certain state is reached.
     */
    public boolean isConditional() {
      return state != null;
    }

    @Override
    public String toString() {
      return name() + " (" + getSpeed() + ", " + getTime() + ", " + getEndState() + ")";
    }
  }

  @Getter
  @Setter
  private static IntakeAction mutableIntakeAction = IntakeAction.STOP;
}