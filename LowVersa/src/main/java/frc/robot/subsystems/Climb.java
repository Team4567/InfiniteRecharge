/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  VictorSPX climb;
  
  /**
   * Creates a new Climb.
   */
  public Climb() {
    climb = new VictorSPX( Constants.kCANClimb );
    climb.configFactoryDefault();
    climb.setNeutralMode( NeutralMode.Brake );
    climb.configPeakOutputForward( +1.0 );
    climb.configPeakOutputReverse( -1.0 );
  }

  public void control( double power ){
    climb.set( ControlMode.PercentOutput, power );
  }
  public void control( DoubleSupplier power ){
    control( power.getAsDouble() );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
