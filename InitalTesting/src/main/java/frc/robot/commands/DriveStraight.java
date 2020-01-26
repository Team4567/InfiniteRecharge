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

public class DriveStraight extends CommandBase {
  /**
   * Creates a new DriveStraight.
   */
  
  Drivetrain drive;
  
  double target;
  double targetAngle;
  double target_sensorUnits;
  public DriveStraight( Drivetrain d, double inches ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements( d );
    drive = d;
    target = inches;
    target_sensorUnits = target * drive.inchesToUnits;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("This is Motion Magic with the Auxiliary PID using the Pigeon yaw.");
		drive.zeroDistance();
		
		/* Determine which slot affects which PID */
		drive.rightMaster.selectProfileSlot( Constants.kSlot_Distanc, Constants.PID_PRIMARY );
		drive.rightMaster.selectProfileSlot( Constants.kSlot_Turning, Constants.PID_TURN );
		
		targetAngle = drive.yaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Calculate targets from gamepad inputs */
	  
	  double target_turn = targetAngle;
	
	  /* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Pigeon */
	  drive.rightMaster.set( ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn );
	  drive.rightSlave.follow( drive.rightMaster );
	  drive.leftMaster.follow( drive.rightMaster, FollowerType.AuxOutput1 );
	  drive.leftSlave.follow( drive.leftMaster );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs( target_sensorUnits - drive.rightMaster.getSelectedSensorPosition() ) < 4 * drive.inchesToUnits;
  }
}
