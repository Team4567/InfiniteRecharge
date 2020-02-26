/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class SixBaller extends SequentialCommandGroup {
  /**
   * Creates a new SixBaller.
   */
  public SixBaller( Drivetrain drive, Intake in, ControlPanel control, Climb climb, Misc misc ) {

    addCommands(
      new DriveStraight( drive, 120 ),
      new RunCommand( () -> in.controlIntake( 0.5 ), in ),
      new WaitCommand( 3 ),
      new InstantCommand( () -> in.controlIntake( 0 ), in ),
      new DriveStraight( drive, -36 ),
      new RunCommand( () -> in.controlFlip( 0.1 ) ),
      new WaitCommand( 1 ),
      new InstantCommand( () -> in.controlFlip( 0 ) ),
      new TurnAngle( drive, 180 - 21 ),
      new DriveStraight( drive, 183 ),
      new TurnAngle( drive, 180 ),
      new RunCommand( () -> in.controlIntake( -0.5 ), in ),
      new DriveStraight( drive, 115 ),
      new RunCommand( () -> in.controlFlip( -0.5 ), in )
    );
    addRequirements( drive, in, control, climb, misc );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
