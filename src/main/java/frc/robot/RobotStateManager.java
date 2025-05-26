package frc.robot;

import java.util.function.Supplier;

import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.FieldPose;
import frc.robot.Constants.RobotStateConstants.IntakeAction;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.Constants.RobotStateConstants.ReefHeight;
import frc.robot.Constants.RobotStateConstants.RobotAction;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@AllArgsConstructor
public class RobotStateManager {
  @Getter @Setter private static RobotAction robotAction;
  @Getter @Setter private static boolean alignForAlgaePickup;
  @Getter @Setter private static FieldPose coralScoringPose;
  @Getter @Setter private static boolean intakeOccupied; // if intake is currrently occupied by some other command

  @Getter @Setter private static ReefHeight reefHeight;
  @Getter @Setter private static boolean readyToScoreReef = false;
  @Getter @Setter private static int reefTagId = -1; // the tag ID of the reef spike that we are currently scoring on

  public static FieldPose getReefPoseByTagId(int tagId, boolean left) {
    switch (tagId) {
        case 21: case 10: // Blue 21, Red 10
            return left ? FieldPose.B : FieldPose.A;
        case 22: case 9:  // Blue 22, Red 9
            return left ? FieldPose.D : FieldPose.C;
        case 17: case 8:  // Blue 17, Red 8
            return left ? FieldPose.F : FieldPose.E;
        case 18: case 7:  // Blue 18, Red 7
            return left ? FieldPose.H : FieldPose.G;
        case 19: case 6:  // Blue 19, Red 6
            return left ? FieldPose.J : FieldPose.I;
        case 20: case 11: // Blue 20, Red 11
            return left ? FieldPose.L : FieldPose.K;
        default:
            return FieldPose.NONE; // No matching tag found
    }
}

  

  public static RobotState getRobotState() {
    return new RobotState(robotAction, reefHeight, alignForAlgaePickup, coralScoringPose);
  }

  public static FieldPose getDesiredFieldPose() {
    return getRobotState().getFieldPose();
  }

  public static ElevatorPosition getDesiredElevatorPosition() {
    return getRobotState().getElevatorPosition();
  }

  public static PivotPosition getDesiredPivotPosition() {
    return getRobotState().getPivotPosition();
  }

  public static IntakeAction getDesiredIntakeAction() {
    if(intakeOccupied) {
      return IntakeAction.OCCUPIED;
    }

    return getRobotState().getIntakeAction();
  }

  public static ElevatorPosition getElevatorPositionForSpecificReefHeight(Supplier<ReefHeight> reefHeight) {
    switch(reefHeight.get()) {
      case L1:
        return ElevatorPosition.L1;
      case L2:
        return ElevatorPosition.L2;
      case L3:
        return ElevatorPosition.L3;
      case L4:
        return ElevatorPosition.L4;
      default:
        return ElevatorPosition.DOWN;
    }
  }

  public static PivotPosition getPivotPositionForSpecificReefHeight(Supplier<ReefHeight> reefHeight) {
    switch(reefHeight.get()) {
      case L1:
        return PivotPosition.L1;
      case L2:
        return PivotPosition.L2;
      case L3:
        return PivotPosition.L3;
      case L4:
        return PivotPosition.L4;
      default:
        return PivotPosition.INTAKE_READY;
    }
  }

  public static IntakeAction getIntakeActionForSpecificReefHeight(Supplier<ReefHeight> reefHeight) {
    switch(reefHeight.get()) {
      case L1:
        return IntakeAction.SCORE_L1;
      case L2:
        return IntakeAction.SCORE_L2;
      case L3:
        return IntakeAction.SCORE_L3;
      case L4:
        return IntakeAction.SCORE_L4;
      default:
        return IntakeAction.NONE;
    }
  }

  public static class RobotState {
    @Getter private final ElevatorPosition elevatorPosition;
    @Getter private final PivotPosition pivotPosition;
    @Getter private final FieldPose fieldPose;
    @Getter private final IntakeAction intakeAction;

    /**
     * Constructor for the RobotState class.
     *
     * @param robotAction The current robot action.
     * @param reefHeight The reef height to score at.
     * @param alignForAlgaePickup Whether to align for algae pickup.
     * @param coralScoringPose The coral scoring pose.
     */
    public RobotState(RobotAction robotAction, ReefHeight reefHeight, boolean alignForAlgaePickup, FieldPose coralScoringPose) {
      switch(robotAction) {
        case SCORE_PROCESSOR: {
          elevatorPosition = ElevatorPosition.PROCESSOR;
          pivotPosition = PivotPosition.PROCESSOR;
          fieldPose = FieldPose.PROCESSOR;
          intakeAction = IntakeAction.SCORE_PROCESSOR;
          break;
        }

        case SCORE_BARGE_LEFT: {
          elevatorPosition = ElevatorPosition.BARGE;
          pivotPosition = PivotPosition.BARGE_START;
          fieldPose = FieldPose.BARGE_LEFT;
          intakeAction = IntakeAction.SCORE_BARGE;
          break;
        }

        case SCORE_BARGE_RIGHT: {
          elevatorPosition = ElevatorPosition.BARGE;
          pivotPosition = PivotPosition.BARGE_START;
          fieldPose = FieldPose.BARGE_RIGHT;
          intakeAction = IntakeAction.SCORE_BARGE;
          break;
        }

        case INTAKE_STATION_LEFT: {
          elevatorPosition = ElevatorPosition.DOWN;
          pivotPosition = PivotPosition.INTAKE_READY;
          fieldPose = FieldPose.STATION_LEFT;
          intakeAction = IntakeAction.INTAKE_CORAL;
          break;
        }

        case INTAKE_STATION_RIGHT: {
          elevatorPosition = ElevatorPosition.DOWN;
          pivotPosition = PivotPosition.INTAKE_READY;
          fieldPose = FieldPose.STATION_RIGHT;
          intakeAction = IntakeAction.INTAKE_CORAL;
          break;
        }

        case REEF_ACTION: {
          if(alignForAlgaePickup) {
            switch(coralScoringPose) {
              case A:
              case B:
              case E:
              case F:
              case I:
              case J:
                elevatorPosition = ElevatorPosition.REEF_ALGA_L2;
                pivotPosition = PivotPosition.REEF_ALGA_L2;
                fieldPose = getCorrespondingPose(coralScoringPose);
                intakeAction = IntakeAction.INTAKE_REEF_ALGA;
                break;
              case C:
              case D:
              case G:
              case H:
              case K:
              case L:
                elevatorPosition = ElevatorPosition.REEF_ALGA_L3;
                pivotPosition = PivotPosition.REEF_ALGA_L3;
                fieldPose = getCorrespondingPose(coralScoringPose);
                intakeAction = IntakeAction.INTAKE_REEF_ALGA;
                break;
              default:
                elevatorPosition = ElevatorPosition.DOWN;
                pivotPosition = PivotPosition.INTAKE_READY;
                fieldPose = FieldPose.A;
                intakeAction = IntakeAction.NONE;
                break;
            }
          } else {
            fieldPose = coralScoringPose;

            switch(reefHeight) {
              case L1: {
                elevatorPosition = ElevatorPosition.L1;
                pivotPosition = PivotPosition.L1;
                intakeAction = IntakeAction.SCORE_L1;
                break;
              }

              case L2: {
                elevatorPosition = ElevatorPosition.L2;
                pivotPosition = PivotPosition.L2;
                intakeAction = IntakeAction.SCORE_L2;
                break;
              }

              case L3: {
                elevatorPosition = ElevatorPosition.L3;
                pivotPosition = PivotPosition.L3;
                intakeAction = IntakeAction.SCORE_L3;
                break;
              }

              case L4: {
                elevatorPosition = ElevatorPosition.L4;
                pivotPosition = PivotPosition.L4;
                intakeAction = IntakeAction.SCORE_L4;
                break;
              }
              default:
                elevatorPosition = ElevatorPosition.DOWN;
                pivotPosition = PivotPosition.INTAKE_READY;
                intakeAction = IntakeAction.NONE;
                break;
            }
          }
          break;
        }
        default: {
          elevatorPosition = ElevatorPosition.DOWN;
          pivotPosition = PivotPosition.INTAKE_READY;
          fieldPose = FieldPose.A;
          intakeAction = IntakeAction.NONE;
          break;
        }
      }
    }

    /**
     * Return the corresponding alga pose for the given reef spike pose.
     *
     * @param currentPose The current reef spike pose.
     * @return Corresponding alga pose.
     */
    public static FieldPose getCorrespondingPose(FieldPose currentPose) {
      switch (currentPose) {
        case A:
          return FieldPose.Z1;
        case B:
          return FieldPose.Z1;
        case C:
          return FieldPose.Z2;
        case D:
          return FieldPose.Z2;
        case E:
          return FieldPose.Z3;
        case F:
          return FieldPose.Z3;
        case G:
          return FieldPose.Z4;
        case H:
          return FieldPose.Z4;
        case I:
          return FieldPose.Z5;
        case J:
          return FieldPose.Z5;
        case K:
          return FieldPose.Z6;
        case L:
          return FieldPose.Z6;
        default:
          return FieldPose.Z1;
      }
    }
  }
}
