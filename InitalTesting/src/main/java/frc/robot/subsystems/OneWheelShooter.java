/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gains;

public class OneWheelShooter extends SubsystemBase {
  /**
   * Creates a new TestSystem.
   */
  // kP = 0.3, RPM = 8000
  public TalonSRX talon;

  public OneWheelShooter() {
    talon = new TalonSRX( Constants.kCANTalonOneWheel );
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative );
    //talon.setSensorPhase( true );
    talon.setInverted(true);
    talon.setNeutralMode( NeutralMode.Coast );
    setRPMPID( Constants.kGains_RPMShtr );
  }

  public void setRPMPID( double kP, double kI, double kD, double kF, int kIzone ){
    talon.config_kP( 0, kP );
    talon.config_kI( 0, kI );
    talon.config_kD( 0, kD );
    talon.config_kF( 0, kF );
    talon.config_IntegralZone( 0, kIzone );
  }

  public void setRPMPID( Gains g ){
    setRPMPID( g.kP, g.kI, g.kD, g.kF, g.kIzone );
  }

  public void RPMDrive( double RPM ){
    talon.set( ControlMode.Velocity, ((RPM*4096)/60)/10 );
  }
  
  public void RPMDrive( DoubleSupplier supply ){
    RPMDrive( supply.getAsDouble() );
  }

  public void shootingVelocity(){
    RPMDrive(8000);
  }

  public void stop(){
    talon.set( ControlMode.PercentOutput, 0 );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( "RPM", ( talon.getSelectedSensorVelocity() / 4096 ) * 600 );
  }
}
