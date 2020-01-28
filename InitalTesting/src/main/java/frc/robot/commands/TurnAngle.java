/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnAngle extends CommandBase {
  /**
   * Creates a new DriveStraight.
   */
  
  Drivetrain drive;
 
  double targetAngle;
  public TurnAngle( Drivetrain d, double degrees ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements( d );
    drive = d;
    targetAngle = degrees;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("This is Angle Alignment");
		
		
		/* Determine which slot affects which PID */
		drive.rightMaster.selectProfileSlot( Constants.kSlot_Distanc, Constants.PID_PRIMARY );
		drive.rightMaster.selectProfileSlot( Constants.kSlot_Turning, Constants.PID_TURN );
		
		
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Calculate targets from gamepad inputs */
	  
	  double target_turn = targetAngle;
	
	  /* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Pigeon */
	  drive.rightMaster.set( ControlMode.PercentOutput, 0, DemandType.AuxPID, target_turn );
	  drive.rightSlave.follow( drive.rightMaster );
	  drive.leftMaster.follow( drive.rightMaster, FollowerType.AuxOutput1 );
	  drive.leftSlave.follow( drive.leftMaster );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs( drive.yaw() - targetAngle ) < 5;
  }
}
