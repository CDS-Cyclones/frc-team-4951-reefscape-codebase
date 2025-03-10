// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.operation.timed;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.intake.Intake;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class IntakeCoralTimedCommand extends Command {
//   private final Intake intakeSubsystem;
//   private final Timer timer;


//   /** Creates a new IntakeCoralTimedCommand. */
//   public IntakeCoralTimedCommand(Intake intakeSubsystem) {
//     this.intakeSubsystem = intakeSubsystem;
//     this.timer = new Timer();

//     addRequirements(this.intakeSubsystem);
//   }


//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }


//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     intakeSubsystem.setSpeed(IntakeConstants.kCoralOuttakeSpeed);
//   }


//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intakeSubsystem.stop();
//   }


//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(IntakeConstants.kCoralIntakeTime);
//   }
// }
