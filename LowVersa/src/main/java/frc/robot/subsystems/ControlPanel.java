/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
public class ControlPanel extends SubsystemBase {
  
  public VictorSPX wheel;
  public DoubleSolenoid piston;

  boolean up = false;

  String colorString;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor( 0.143, 0.427, 0.429 );
  private final Color kGreenTarget = ColorMatch.makeColor( 0.197, 0.561, 0.240 );
  private final Color kRedTarget = ColorMatch.makeColor( 0.561, 0.232, 0.114 );
  private final Color kYellowTarget = ColorMatch.makeColor( 0.361, 0.524, 0.113 );
  
  /**
   * Creates a new ControlPanel.
   */
  public ControlPanel() {
    wheel = new VictorSPX( Constants.kCANVictorControl );
    piston = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMControlIn, Constants.kPCMControlOut );
    wheel.configFactoryDefault();
    wheel.setNeutralMode( NeutralMode.Brake );
    wheel.setInverted( false );
    wheel.configPeakOutputForward( +1.0 );
    wheel.configPeakOutputReverse( -1.0 );
    wheel.configNeutralDeadband( Constants.kNeutralDeadband );

    colorSensor = new ColorSensorV3( i2cPort );
    colorMatcher.addColorMatch( kBlueTarget );
    colorMatcher.addColorMatch( kGreenTarget );
    colorMatcher.addColorMatch( kRedTarget );
    colorMatcher.addColorMatch( kYellowTarget );  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    
    ColorMatchResult match = colorMatcher.matchClosestColor( detectedColor );

    if ( match.color == kBlueTarget) {
      colorString = "Blue";
    } else if ( match.color == kRedTarget ) {
      colorString = "Red";
    } else if ( match.color == kGreenTarget ) {
      colorString = "Green";
    } else if ( match.color == kYellowTarget ) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber( "Red", detectedColor.red );
    SmartDashboard.putNumber( "Green", detectedColor.green );
    SmartDashboard.putNumber( "Blue", detectedColor.blue );
    SmartDashboard.putNumber( "Confidence", match.confidence );
    SmartDashboard.putString( "Detected Color", colorString );

    SmartDashboard.putString( "GameData", getGameMessage() );
  }

  public void control( double power ){
    wheel.set( ControlMode.PercentOutput, power );
  }
  
  public void control( DoubleSupplier power ){
    control( power.getAsDouble() );
  }

  public void toggle(){
    piston.set( up ? Value.kForward : Value.kReverse );
    up = !up;
  }

  public void stop(){
    control( 0 );
  }

  public String getGameMessage(){
    String out = DriverStation.getInstance().getGameSpecificMessage();
    if( out.length() == 0 ){
      return "N/A";
    }else{
      return out;
    }
  }

  public void setToColor( ){
    char c = getGameMessage().charAt(0);
    // Positive = clockwise
    // Negative = counter-Clockwise
    if( c == 'R' ){
      if( colorString == "Blue" ){
        control( 0 );
      }else if( colorString == "Red" ){
        control( 0.5 );
      }else if( colorString == "Green" ){
        control(0.5);
      }else if( colorString == "Yellow" ){
        control( -0.5 );
      }
    }else if( c == 'G' ){
      if( colorString == "Blue" ){
        control( 0.5 );
      }else if( colorString == "Red" ){
        control( -0.5 );
      }else if( colorString == "Green" ){
        control( 0.5 );
      }else if( colorString == "Yellow" ){
        control( 0 );
      }
    }else if( c == 'B' ){
      if( colorString == "Blue" ){
        control( 0.5 );
      }else if( colorString == "Red" ){
        control( 0 );
      }else if( colorString == "Green" ){
        control( -0.5 );
      }else if( colorString == "Yellow" ){
        control( 0.5 );
      }
    }else if( c == 'Y' ){
      if( colorString == "Blue" ){
        control( -0.5 );
      }else if( colorString == "Red" ){
        control( 0.5 );
      }else if( colorString == "Green" ){
        control( 0 );
      }else if( colorString == "Yellow" ){
        control( 0.5 );
      }
    }
  }
}
