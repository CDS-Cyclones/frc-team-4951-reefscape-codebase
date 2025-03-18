package frc.robot;

import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.FieldPose;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.Constants.RobotStateConstants.ReefHeight;
import frc.robot.Constants.RobotStateConstants.RobotAction;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@AllArgsConstructor
public class RobotStateManager {
  @Setter private static RobotAction robotAction;
  @Setter private static ReefHeight reefHeight;
  @Setter private static boolean alignForAlgaePickup;
  @Setter private static FieldPose coralScoringPose;

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

  public static class RobotState {
    @Getter private final ElevatorPosition elevatorPosition;
    @Getter private final PivotPosition pivotPosition;
    @Getter private final FieldPose fieldPose;

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
          break;
        }

        case SCORE_BARGE_LEFT: {
          elevatorPosition = ElevatorPosition.BARGE;
          pivotPosition = PivotPosition.BARGE;
          fieldPose = FieldPose.BARGE_LEFT;
          break;
        }

        case SCORE_BARGE_RIGHT: {
          elevatorPosition = ElevatorPosition.BARGE;
          pivotPosition = PivotPosition.BARGE;
          fieldPose = FieldPose.BARGE_RIGHT;
          break;
        }

        case INTAKE_STATION_LEFT: {
          elevatorPosition = ElevatorPosition.DOWN;
          pivotPosition = PivotPosition.INTAKE_READY;
          fieldPose = FieldPose.STATION_LEFT;
          break;
        }

        case INTAKE_STATION_RIGHT: {
          elevatorPosition = ElevatorPosition.DOWN;
          pivotPosition = PivotPosition.INTAKE_READY;
          fieldPose = FieldPose.STATION_RIGHT;
          break;
        }

        case REEF_ACTION: {
          if(alignForAlgaePickup) {
            elevatorPosition = ElevatorPosition.REEF_ALGA;
            pivotPosition = PivotPosition.REEF_ALGA;
            fieldPose = getCorrespondingPose(coralScoringPose);
          } else {
            fieldPose = coralScoringPose;

            switch(reefHeight) {
              case L1: {
                elevatorPosition = ElevatorPosition.L1;
                pivotPosition = PivotPosition.L1;
                break;
              }

              case L2: {
                elevatorPosition = ElevatorPosition.L2;
                pivotPosition = PivotPosition.L2;
                break;
              }

              case L3: {
                elevatorPosition = ElevatorPosition.L3;
                pivotPosition = PivotPosition.L3;
                break;
              }

              case L4: {
                elevatorPosition = ElevatorPosition.L4;
                pivotPosition = PivotPosition.L4;
                break;
              }
              default:
                elevatorPosition = ElevatorPosition.DOWN;
                pivotPosition = PivotPosition.INTAKE_READY;
                break;
            }
          }
          break;
        }
        default: {
          elevatorPosition = ElevatorPosition.DOWN;
          pivotPosition = PivotPosition.INTAKE_READY;
          fieldPose = FieldPose.A;
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
    private static FieldPose getCorrespondingPose(FieldPose currentPose) {
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
