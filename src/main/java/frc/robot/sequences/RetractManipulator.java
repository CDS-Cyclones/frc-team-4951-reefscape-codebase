package frc.robot.sequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.commands.elevator.ElevatorToPositionCommand;
import frc.robot.commands.pivot.PivotToPositionCommand;
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
        new PivotToPositionCommand(pivot, () -> PivotPosition.ELEVATOR_CLEAR),
        Commands.none(),
        () -> pivot.getPosition() < PivotPosition.ELEVATOR_CLEAR.getAsDouble()
      ),
      Commands.parallel(
        new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.DOWN),
        new PivotToPositionCommand(pivot, () -> PivotPosition.ELEVATOR_CLEAR)
      ),
      new PivotToPositionCommand(pivot, () -> PivotPosition.INTAKE_READY)
    );
  }
}