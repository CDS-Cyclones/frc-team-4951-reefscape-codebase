package frc.robot.sequences;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class PositionManipulator extends SequentialCommandGroup {
  /**
   * Positions elevator and pivot for specific task(score coral, barge, processor, etc.).
   * 
   * @param elevator
   * @param pivot
   * @param elevatorPositionSupplier
   * @param pivotPositionSupplier
   */
  public PositionManipulator(Elevator elevator, Pivot pivot, Supplier<ElevatorPosition> elevatorPositionSupplier, Supplier<PivotPosition> pivotPositionSupplier) {
    addCommands(
      new ConditionalCommand( // If pivot is in elevator's way, move it out of the way before raising
        Commands.sequence(
          pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
          Commands.waitUntil(() -> pivot.isAtPosition(PivotPosition.ELEVATOR_CLEAR))
        ),
        Commands.none(),
        () -> !pivot.isOutOfElevatorWay()
      ),
      elevator.moveToPosition(pivot, elevatorPositionSupplier),
      Commands.waitUntil(() -> elevator.isAtPosition(elevatorPositionSupplier.get())),
      pivot.moveToPosition(pivotPositionSupplier),
      Commands.waitUntil(() -> pivot.isAtPosition(pivotPositionSupplier.get()))
    );
  }
}