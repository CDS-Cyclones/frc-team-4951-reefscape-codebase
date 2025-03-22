package frc.robot.sequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class RetractManipulator extends SequentialCommandGroup {
  /**
   * Retract elevator and pivot fully.
   * 
   * @param elevator
   * @param pivot
   */
  public RetractManipulator(Elevator elevator, Pivot pivot) {
    addCommands(
      new ConditionalCommand( // If pivot is in elevator's way, move it out of the way before lowering
        Commands.sequence(
          pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
          Commands.waitUntil(() -> pivot.isAtPosition(PivotPosition.ELEVATOR_CLEAR))
        ),
        Commands.none(),
        () -> !pivot.isOutOfElevatorWay()
      ),
      Commands.parallel(
        elevator.moveToPosition(pivot, () -> ElevatorPosition.DOWN),
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR)
      ),
      Commands.waitUntil(() -> elevator.isAtPosition(ElevatorPosition.DOWN) && pivot.isAtPosition(PivotPosition.ELEVATOR_CLEAR)),
      pivot.moveToPosition(() -> PivotPosition.INTAKE_READY),
      Commands.waitUntil(() -> pivot.isAtPosition(PivotPosition.INTAKE_READY))
    );
  }
}