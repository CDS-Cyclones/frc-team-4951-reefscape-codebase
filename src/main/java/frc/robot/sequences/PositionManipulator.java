package frc.robot.sequences;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorToPositionCommand;
import frc.robot.commands.pivot.PivotToPositionCommand;
import frc.robot.mutables.MutableElevatorPosition.ElevatorPosition;
import frc.robot.mutables.MutablePivotPosition.PivotPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

public class PositionManipulator extends SequentialCommandGroup {
  /**
   * Positions elevator and pivot for specific task(score coral, barge, processor, etc.).
   * 
   * @param elevator
   * @param pivot
   */
  public PositionManipulator(Elevator elevator, Pivot pivot, Supplier<ElevatorPosition> elevatorPositionSupplier, Supplier<PivotPosition> pivotPositionSupplier) {
    addCommands(
      new ConditionalCommand( // If pivot is in elevator's way, move it out of the way before raising
        new PivotToPositionCommand(pivot, () -> PivotPosition.ELEVATOR_CLEAR),
        Commands.none(),
        () -> pivot.getPosition() < PivotPosition.ELEVATOR_CLEAR.getAsDouble()
      ),
      Commands.parallel( // Get elevator and pivot to scoring positions
        new ElevatorToPositionCommand(elevator, pivot, elevatorPositionSupplier),
        new PivotToPositionCommand(pivot, pivotPositionSupplier)
      )
    );
  }
}