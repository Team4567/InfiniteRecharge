/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.Target;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drive;
  public final OneWheelShooter shoot;
  public final ControlPanel control;

  public final Compressor compressor;
  private Command m_autoCommand = null;   

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table,limelight;
  NetworkTableEntry tx, ty, targetVisible, rpm;

  BobController controller = new BobController(0);

  double angleLimelight = 0;
  double heightLimelight = 0;
  // Center of target
  double heightTarget = 89.75;
  

  SendableChooser<CommandGroupBase> chooser;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drivetrain();
    shoot = new OneWheelShooter();
    control = new ControlPanel();
    compressor = new Compressor( Constants.kCANPCMA );
    compressor.setClosedLoopControl(true);
    compressor.start();

    limelight = inst.getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    targetVisible = limelight.getEntry("tv");

    table = inst.getTable( "Table" );

    rpm = table.getEntry("RPM");
    rpm.setDouble(8000);

    //m_autoCommand = new DriveStraight( drive, 120 );

    chooser = new SendableChooser<CommandGroupBase>();
    chooser.setDefaultOption("Auto A", new Target(0, 0, drive));
    chooser.addOption("Auto B", new Target(0, 0, drive));
    chooser.addOption("Auto C", new Target(0, 0, drive));
    chooser.addOption("Test", new SequentialCommandGroup( new TurnAngle( drive, 0 ), new DriveStraight( drive, 0 ) ) );
    SmartDashboard.putData( "Auto Command", chooser );
    


    // Configure the button bindings
    configureButtonBindings();
  }

  public double getTargetRPM(){
    return rpm.getDouble(0);
  }

  public boolean haveTarget(){
      return targetVisible.getDouble( 0 ) == 1;
  }

  public double getDistanceToTarget(){
      return haveTarget() ? ( heightTarget - heightLimelight ) / Math.tan( angleLimelight + ty.getDouble( 0 ) ) : 0;
  }

  public double getAngleToTarget(){
      return haveTarget() ? tx.getDouble( 0 ) : 0;
  }

  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      drive.setDefaultCommand( new RunCommand( () -> drive.drive( controller::getLeftStickY, controller::getLeftStickX ) , drive ) );
      shoot.setDefaultCommand( new RunCommand( () -> shoot.talon.set( ControlMode.PercentOutput, controller.getRightTrigger() ), shoot ) );
      control.setDefaultCommand( new RunCommand( () -> control.control( controller.getRightStickX() ), control ) );
      
      controller.aButton.whenHeld( new RunCommand( () -> shoot.RPMDrive( 8000 ), shoot  ) );
        //.whenReleased( new InstantCommand( shoot::stop, shoot ) );
      controller.bButton.whenPressed( haveTarget() 
        ? new SequentialCommandGroup( new TurnAngle( drive, getAngleToTarget() ), new DriveStraight( drive, getDistanceToTarget() ) ) 
        : new PrintCommand("No Target!") );

      controller.rightBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.HighGear ) ) );
      controller.leftBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.LowGear ) ) );
      
      controller.startButton.whenHeld( new RunCommand( control::setToColor, control ) );
        //.whenReleased( new InstantCommand( control::stop, control ) );
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }
}
