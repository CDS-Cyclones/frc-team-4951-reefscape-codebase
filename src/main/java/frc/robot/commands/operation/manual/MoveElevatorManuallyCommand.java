// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.operation.manual;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.pivot.Pivot;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class MoveElevatorManuallyCommand extends Command {
//   private final Elevator elevator;
//   private final PivotOld pivotSubsystem;
//   private final boolean moveUp;

//   /** Creates a new MoveElevatorManuallyCommand. */
//   public MoveElevatorManuallyCommand(Elevator elevatorSubsystem, PivotOld pivotSubsystem, boolean moveUp) {
//     this.elevatorSubsystem = elevatorSubsystem;
//     this.pivotSubsystem = pivotSubsystem;
//     this.moveUp = moveUp;

//     addRequirements(this.elevatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(pivotSubsystem.isOut()) { // assure that pivot is out
//       if(moveUp) {
//         elevatorSubsystem.setSpeed(0.25);
//       } else {
//         elevatorSubsystem.setSpeed(-0.3);
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     elevatorSubsystem.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
