/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IMU;

public class TestSubsystem extends SubsystemBase {
  TalonSRX talon;
  IMU imu;
  PowerDistributionPanel pdp;
  DigitalInput downLimit, upLimit;
  /**
   * Creates a new ExampleSubsystem.
   */
  public TestSubsystem() {
    talon = new TalonSRX( 1 );
    talon.setInverted(true);
    imu = new IMU( 5 );
    pdp = new PowerDistributionPanel( 0 );
    downLimit = new DigitalInput( 0 );
    upLimit = new DigitalInput( 1 );
  }

  public void operate( double value, double mult ){
    double input = value*mult;
    if( !( ( downLimit.get() && input > 0 ) || ( upLimit.get() && input < 0 ) ) ){
      talon.set( ControlMode.PercentOutput, input);
    } else{
      talon.set( ControlMode.PercentOutput, 0 );
    }
  }

  public void operate( double value ){
    operate( value, 1 );
  }

  public void operate( DoubleSupplier value, double mult ){
    operate( value.getAsDouble(), mult );
  }
  
  public void operate( DoubleSupplier value ){
    operate( value, 1 );
  }

  public boolean isUp(){
    return upLimit.get();
  }

  public boolean isDown(){
    return downLimit.get();
  }
  @Override
  public void periodic() {
    SmartDashboard.putData( imu );
    SmartDashboard.putData( pdp );
    SmartDashboard.putBoolean( "Down", downLimit.get() );
    SmartDashboard.putBoolean( "Up", upLimit.get() );
    // This method will be called once per scheduler run
  }
}
