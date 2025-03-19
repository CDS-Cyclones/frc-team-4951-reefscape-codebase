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
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        Commands.none(),
        () -> pivot.getPosition() < PivotPosition.ELEVATOR_CLEAR.getAsDouble()
      ),
      Commands.parallel(
        Commands.runOnce(() -> elevator.setReference(ElevatorPosition.DOWN)),
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR)
      ),
      pivot.moveToPosition(() -> PivotPosition.INTAKE_READY)
    );
  }
}