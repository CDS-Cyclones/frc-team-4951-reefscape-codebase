package frc.robot.sequences;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      Commands.runOnce(()-> SmartDashboard.putBoolean("CommandOver",false)),
      new ConditionalCommand( // If pivot is in elevator's way, move it out of the way before raising
        Commands.sequence(
          pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
          Commands.waitUntil(() -> pivot.isAtPosition(PivotPosition.ELEVATOR_CLEAR))
        ),
        Commands.none(),
        () -> !pivot.isOutOfElevatorWay()
      ),
      Commands.parallel( // Get elevator and pivot to scoring positions
        elevator.moveToPosition(pivot, elevatorPositionSupplier),
        pivot.moveToPosition(pivotPositionSupplier)
      ),
      Commands.waitUntil(() -> elevator.isAtPosition(elevatorPositionSupplier.get()) && pivot.isAtPosition(pivotPositionSupplier.get())),
      Commands.runOnce(()-> SmartDashboard.putBoolean("CommandOver",true))
    );
  }
}