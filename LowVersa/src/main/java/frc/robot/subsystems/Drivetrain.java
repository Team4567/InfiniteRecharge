/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IMU;

public class Drivetrain extends SubsystemBase {
  /**
	 * Defines the gear of the dual-speed gearbox
	 */
  public static enum Gear{
	  HighGear,
	  LowGear
  }
  // double : 1
  final double lowGearRatio  = 15.32;
  final double highGearRatio = 7.08;
  final double cpr = 2048;
  final double wheelCirc = 6*Math.PI;
  public double inchesToUnits = 1;
  public TalonFX leftMaster, leftSlave, rightMaster, rightSlave;
  public IMU imu;
  public DoubleSolenoid gearShift;
  public Gear gear, prevGear;
  public Orchestra meme; 
  private boolean playing = false;
  /** Tracking variables */
	double targetAngle = 0;
	  double[] ypr = {0,0,0};
	  double prevY = 0;
	/**
   * Creates a new Drivetrain.
   */
	public Drivetrain() {
    	leftMaster = new TalonFX( Constants.kCANLMaster );
    	leftSlave = new TalonFX( Constants.kCANLSlave );
    	rightMaster = new TalonFX( Constants.kCANRMaster );
		rightSlave = new TalonFX( Constants.kCANRSlave );
	
		imu = new IMU( Constants.kCANIMU );

		gearShift = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMLGearboxIn, Constants.kPCMLGearboxOut );
		
		meme = new Orchestra();

		meme.addInstrument( leftMaster );
		meme.addInstrument( leftSlave );
		meme.addInstrument( rightMaster );
		meme.addInstrument( rightSlave );
		meme.loadMusic("Cantina.chrp");
		meme.stop();
		

		
    /* Factory Default all hardware to prevent unexpected behavior */
		rightMaster.configFactoryDefault();
		rightSlave.configFactoryDefault();
		leftMaster.configFactoryDefault();
		leftSlave.configFactoryDefault();

		imu.configFactoryDefault();
		
		/* Set Neutral Mode */
		leftMaster.setNeutralMode( NeutralMode.Brake );
		leftSlave.setNeutralMode( NeutralMode.Brake );
		rightMaster.setNeutralMode( NeutralMode.Brake );
		rightSlave.setNeutralMode( NeutralMode.Brake );
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		leftMaster.configSelectedFeedbackSensor(	TalonFXFeedbackDevice.IntegratedSensor,				// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs );					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		rightMaster.configRemoteFeedbackFilter(		leftMaster.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonFX_SelectedSensor,	// Remote Feedback Source
												Constants.REMOTE_0,							// Source number [0, 1]
												Constants.kTimeoutMs );						// Configuration Timeout
		
		/* Configure the Pigeon IMU to the other remote slot available on the right Talon */
		rightMaster.configRemoteFeedbackFilter(		imu.getDeviceID(),
												RemoteSensorSource.Pigeon_Yaw,
												Constants.REMOTE_1,	
												Constants.kTimeoutMs );
		
		/* Setup Sum signal to be used for Distance */
		rightMaster.configSensorTerm( SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs );				// Feedback Device of Remote Talon
		rightMaster.configSensorTerm( SensorTerm.Sum1, FeedbackDevice.IntegratedSensor, Constants.kTimeoutMs );	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs );
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs );		// Configuration Timeout
		
		/* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
													Constants.PID_TURN,
													Constants.kTimeoutMs );
		
		/* Scale the Feedback Sensor using a coefficient */
		rightMaster.configSelectedFeedbackCoefficient(	1,
														Constants.PID_TURN,
														Constants.kTimeoutMs );
		
		/* Configure output and sensor direction */
		leftMaster.setInverted( false );
		leftMaster.setSensorPhase( true );
		leftSlave.setInverted( false );
		leftSlave.setSensorPhase( true );

		rightMaster.setInverted( true );
		rightMaster.setSensorPhase( true );
		rightSlave.setInverted( true );
	  	rightSlave.setSensorPhase( true );
		
		/* Set status frame periods to ensure we don't have stale data */
		rightMaster.setStatusFramePeriod( StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs );
		rightMaster.setStatusFramePeriod( StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs );
		rightMaster.setStatusFramePeriod( StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs );
		rightMaster.setStatusFramePeriod( StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs );
		leftMaster.setStatusFramePeriod( StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs );

		imu.setStatusFramePeriod( PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, Constants.kTimeoutMs );

		/* Configure neutral deadband */
		rightMaster.configNeutralDeadband( Constants.kNeutralDeadband, Constants.kTimeoutMs );
		leftMaster.configNeutralDeadband( Constants.kNeutralDeadband, Constants.kTimeoutMs );
		rightSlave.configNeutralDeadband( Constants.kNeutralDeadband, Constants.kTimeoutMs );
		leftSlave.configNeutralDeadband( Constants.kNeutralDeadband, Constants.kTimeoutMs );
		
		/* Motion Magic Configurations */
		rightMaster.configMotionAcceleration( 2000, Constants.kTimeoutMs );
		rightMaster.configMotionCruiseVelocity( 2000, Constants.kTimeoutMs );

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		leftMaster.configPeakOutputForward( +1.0, Constants.kTimeoutMs );
		leftMaster.configPeakOutputReverse( -1.0, Constants.kTimeoutMs );
		rightMaster.configPeakOutputForward( +1.0, Constants.kTimeoutMs );
		rightMaster.configPeakOutputReverse( -1.0, Constants.kTimeoutMs );
		leftSlave.configPeakOutputForward( +1.0, Constants.kTimeoutMs );
		leftSlave.configPeakOutputReverse( -1.0, Constants.kTimeoutMs );
		rightSlave.configPeakOutputForward( +1.0, Constants.kTimeoutMs );
		rightSlave.configPeakOutputReverse( -1.0, Constants.kTimeoutMs );

		/* FPID Gains for distance servo */
		rightMaster.config_kP( Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs );
		rightMaster.config_kI( Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs );
		rightMaster.config_kD( Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs );
		rightMaster.config_kF( Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs );
		rightMaster.config_IntegralZone( Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs );
		rightMaster.configClosedLoopPeakOutput( Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs );

		/* FPID Gains for turn servo */
		rightMaster.config_kP( Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs );
		rightMaster.config_kI( Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs );
		rightMaster.config_kD( Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs );
		rightMaster.config_kF( Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs );
		rightMaster.config_IntegralZone( Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs );
		rightMaster.configClosedLoopPeakOutput( Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs );
		
		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		rightMaster.configClosedLoopPeriod( 0, closedLoopTimeMs, Constants.kTimeoutMs );
		rightMaster.configClosedLoopPeriod( 1, closedLoopTimeMs, Constants.kTimeoutMs );

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		rightMaster.configAuxPIDPolarity( false, Constants.kTimeoutMs );

		/* Initialize */
		rightMaster.setStatusFramePeriod( StatusFrameEnhanced.Status_10_Targets, 10 );
		
		setGear( Gear.LowGear );

		zeroSensors();
		
  }
  
  public double yaw(){
	return ypr[2] % 360;
  }
  
  public void stop(){
	rightMaster.set( ControlMode.PercentOutput, 0 );
	rightSlave.set( ControlMode.PercentOutput, 0 );
	leftMaster.set( ControlMode.PercentOutput, 0 );
	leftSlave.set( ControlMode.PercentOutput, 0 );
  }

  /**
   * @param gear the gear to set
   */
  public void setGear( Gear gear ) {
      prevGear = this.gear;  
	  this.gear = gear;
  }
  public Gear getGear(){
	  return gear;
  }
  public String getGearString(){
	  if( gear == Gear.HighGear ){
		return "High";
	  }else if( gear == Gear.LowGear ){
		return "Low";
	  }else{
		  return "N/A";
	  }
  }

  public void toggleSound(){
	if( playing ){
		meme.stop();
	}else{
		meme.play();
	}
	playing = !playing;
  }

  @Override
  public void periodic() {
		if( gear == Gear.LowGear ){
			inchesToUnits = cpr * lowGearRatio / wheelCirc;
			gearShift.set( DoubleSolenoid.Value.kForward );
	  }else if( gear == Gear.HighGear ){
			inchesToUnits = cpr * highGearRatio / wheelCirc;
			gearShift.set( DoubleSolenoid.Value.kReverse );
		}
	  SmartDashboard.putString( "Gear", getGearString() );
	  SmartDashboard.putData( "Gyro", imu );
	  
	  imu.getYawPitchRoll( ypr );
    // This method will be called once per scheduler run
  }

  public void drive( double y, double x ){
	
		y = Math.max( -1, Math.min( y, 1 ) );
		y = ( Math.abs( y - prevY ) > 0.2 ) ? prevY + Math.copySign( 0.2, y - prevY ) : y;
    	x = Math.max( -1, Math.min( x, 1 ) );
    

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
      //y = Math.copySign( y * y, y );
      //x = Math.copySign( x * x, x );

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign( Math.max( Math.abs( y ), Math.abs( x ) ), y );

    if ( y >= 0.0 ) {
      // First quadrant, else second quadrant
      if ( x >= 0.0 ) {
        leftMotorOutput = maxInput;
        rightMotorOutput = y - x;
      } else {
        leftMotorOutput = y + x;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if ( x >= 0.0 ) {
        leftMotorOutput = y + x;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = y - x;
      }
	}
	
	
	leftMaster.set( ControlMode.PercentOutput, leftMotorOutput );
	leftSlave.follow( leftMaster );
	rightMaster.set( ControlMode.PercentOutput, rightMotorOutput );
	rightSlave.follow( rightMaster );
	//leftSlave.set( ControlMode.PercentOutput, leftMotorOutput );
	//rightSlave.set( ControlMode.PercentOutput, rightMotorOutput );
	prevY = y;
}
  
  public void drive( DoubleSupplier y, DoubleSupplier x ){
	drive( y.getAsDouble(), x.getAsDouble() );
  }

  public void music(){
	  leftMaster.set( ControlMode.MusicTone, 0 );
	  leftSlave.set( ControlMode.MusicTone, 0 );
	  rightMaster.set( ControlMode.MusicTone, 0 );
	  rightSlave.set( ControlMode.MusicTone, 0 );
  }
  /** Zero all sensors, both Talons and Pigeon */
	void zeroSensors() {
		leftMaster.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
		rightMaster.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
		leftSlave.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
    	rightSlave.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
  
		imu.setYaw( 0, Constants.kTimeoutMs );
		imu.setAccumZAngle( 0, Constants.kTimeoutMs );
		System.out.println( "[Quadrature Encoders + Pigeon] All sensors are zeroed.\n" );
	}
	
	public void zeroDistance(){
		leftMaster.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
		rightMaster.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
		leftSlave.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
    	rightSlave.getSensorCollection().setIntegratedSensorPosition( 0, Constants.kTimeoutMs );
		System.out.println( "[Quadrature Encoders] All encoders are zeroed.\n" );
	}
}
