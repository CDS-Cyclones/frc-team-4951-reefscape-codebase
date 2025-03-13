package frc.robot.sequences;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorToPositionCommand;
import frc.robot.commands.intake.ScoreCoralCommand;
import frc.robot.commands.pivot.PivotToPositionCommand;
import frc.robot.mutables.MutableElevatorPosition;
import frc.robot.mutables.MutableElevatorPosition.ElevatorPosition;
import frc.robot.mutables.MutablePivotPosition.PivotPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class ScoreSequence extends SequentialCommandGroup {
  public ScoreSequence(Elevator elevator, Pivot pivot, Intake intake) {
    addCommands(
    );
  }
}