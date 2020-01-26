/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotDrive;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainTalon extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  public WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
  public PigeonIMU imu;
  public Compressor compressor;
  // kP = 0.3, RPM = 8000
  /** Tracking variables */
	boolean firstCall = false;
	boolean state = false;
	double targetAngle = 0;
	double privY = 0;
  public DrivetrainTalon() {
    leftMaster = new WPI_TalonSRX(Constants.kCANLMaster);
    leftSlave = new WPI_TalonSRX(Constants.kCANLSlave);
    rightMaster = new WPI_TalonSRX(Constants.kCANRMaster);
	rightSlave = new WPI_TalonSRX(Constants.kCANRSlave);
	imu = new PigeonIMU(leftSlave);
	
    /* Factory Default all hardware to prevent unexpected behavior */
	rightMaster.configFactoryDefault();
	rightSlave.configFactoryDefault();
	leftMaster.configFactoryDefault();
	leftSlave.configFactoryDefault();
	imu.configFactoryDefault();
	
	/* Set Neutral Mode */
	leftMaster.setNeutralMode(NeutralMode.Brake);
	rightMaster.setNeutralMode(NeutralMode.Brake);
	leftSlave.setNeutralMode(NeutralMode.Brake);
	rightSlave.setNeutralMode(NeutralMode.Brake);

	leftMaster.configNeutralDeadband(0.1);
	rightMaster.configNeutralDeadband(0.1);
	leftSlave.configNeutralDeadband(0.1);
	rightSlave.configNeutralDeadband(0.1);
	
	/** Feedback Sensor Configuration */
	
	/* Configure the left Talon's selected sensor as local QuadEncoder */
	leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative,				// Local Feedback Source
												Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
												Constants.kTimeoutMs);					// Configuration Timeout

	/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
	rightMaster.configRemoteFeedbackFilter(leftMaster.getDeviceID(),					// Device ID of Source
											RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
											Constants.REMOTE_0,							// Source number [0, 1]
											Constants.kTimeoutMs);						// Configuration Timeout
	
	/* Configure the Pigeon IMU to the other remote slot available on the right Talon */
	rightMaster.configRemoteFeedbackFilter(imu.getDeviceID(),
											RemoteSensorSource.GadgeteerPigeon_Yaw,
											Constants.REMOTE_1,	
											Constants.kTimeoutMs);
	
	/* Setup Sum signal to be used for Distance */
	rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
	rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
	
	/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
	rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
												Constants.PID_PRIMARY,
												Constants.kTimeoutMs);
	
	/* Scale Feedback by 0.5 to half the sum of Distance */
	rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
													Constants.PID_PRIMARY,		// PID Slot of Source 
													Constants.kTimeoutMs);		// Configuration Timeout
	
	/* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
	rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
												Constants.PID_TURN,
												Constants.kTimeoutMs);
	
	/* Scale the Feedback Sensor using a coefficient */
	rightMaster.configSelectedFeedbackCoefficient(	1,
													Constants.PID_TURN,
													Constants.kTimeoutMs);
	
	/* Configure output and sensor direction */
	leftMaster.setInverted(false);
	leftMaster.setSensorPhase(true);
	rightMaster.setInverted(true);
	rightMaster.setSensorPhase(true);
	rightSlave.setInverted(true);
	leftSlave.setInverted(false);
	
	/* Set status frame periods to ensure we don't have stale data */
	/*rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
	rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
	rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
	rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
	leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
	imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, Constants.kTimeoutMs);
*/
	/* Configure neutral deadband */
//	rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
//	leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
	
	/* Motion Magic Configurations */
	rightMaster.configMotionAcceleration(2000, Constants.kTimeoutMs);
	rightMaster.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

	/**
	 * Max out the peak output (for all modes).  
	 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
	 */
	leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
	leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
	rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
	rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

	leftSlave.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
	leftSlave.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
	rightSlave.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
	rightSlave.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

	/* FPID Gains for distance servo */
	rightMaster.config_kP(0, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
	rightMaster.config_kI(0, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
	rightMaster.config_kD(0, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
	rightMaster.config_kF(0, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
	rightMaster.config_IntegralZone(0, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
	rightMaster.configClosedLoopPeakOutput(0, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);

	/* FPID Gains for turn servo */
	rightMaster.config_kP(1, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
	rightMaster.config_kI(1, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
	rightMaster.config_kD(1, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
	rightMaster.config_kF(1, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
	rightMaster.config_IntegralZone(1, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
	rightMaster.configClosedLoopPeakOutput(1, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
	
	/**
	 * 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	int closedLoopTimeMs = 1;
	rightMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
	rightMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

	/**
	 * configAuxPIDPolarity(boolean invert, int timeoutMs)
	 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
	 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
	 */
	rightMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);

	/* Initialize */
	firstCall = true;
	state = false;
	rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
	zeroSensors();
		
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDistanceTarget( double inches ){

  }

  public void driveStraight( double target ){
	leftMaster.follow(rightMaster, FollowerType.PercentOutput);
	rightMaster.set( ControlMode.MotionMagic, target, DemandType.AuxPID, 0);
	leftSlave.follow( leftMaster, FollowerType.PercentOutput );
	rightSlave.follow( rightMaster, FollowerType.PercentOutput );
  }

  public void drive( double yee, double x ){
	double y = yee;
	if( Math.abs( y - privY ) > .02 ){
	   y = privY + ( Math.signum( y - privY ) * .02 );
	}
	
	double leftMotors,rightMotors;
	if ( y > 0.0 ) {
	  if ( x > 0.0 ) {
		leftMotors = y - x;
		rightMotors = Math.max( y, x );
	  } else {
		leftMotors = Math.max( y, -x );
		rightMotors = y + x;
	  }
	} else {
	  if ( x > 0.0 ) {
		leftMotors = -Math.max( -y, x );
		rightMotors = y + x;
	  } else {
		leftMotors = y - x;
		rightMotors = -Math.max( -y, -x );
	  }
	  
	}
	
	rightMaster.set( ControlMode.PercentOutput, rightMotors );
	leftMaster.set( ControlMode.PercentOutput, leftMotors );
	rightSlave.follow( rightMaster );
	leftSlave.follow( leftMaster );
	privY = y;
  }
  /** Zero all sensors, both Talons and Pigeon */
	void zeroSensors() {
		leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		imu.setYaw(0, Constants.kTimeoutMs);
		imu.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders + Pigeon] All sensors are zeroed.\n");
	}
	/** Zero QuadEncoders, used to reset position when initializing Motion Magic */
	void zeroDistance(){
		leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders] All encoders are zeroed.\n");
	}
		
}
