/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

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

  private Command m_autoCommand = null;   

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table,limelight;
  NetworkTableEntry tx, ty, targetVisible, rpm;

  BobController controller = new BobController(0);

  double angleLimelight = 0;
  double heightLimelight = 0;
  // Center of target
  double heightTarget = 89.75;
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drivetrain();
    shoot = new OneWheelShooter();
    control = new ControlPanel();

    limelight = inst.getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    targetVisible = limelight.getEntry("tv");

    table = inst.getTable( "Table" );

    rpm = table.getEntry("RPM");
    rpm.setDouble(8000);

    //m_autoCommand = new DriveStraight( drive, 120 );

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
      
      controller.aButton.whenPressed( new InstantCommand( () -> shoot.RPMDrive( 8000 ), shoot  ) )
        .whenReleased( new InstantCommand( shoot::stop, shoot  ) );
      controller.bButton.whenPressed( new AlignToTarget() );

      controller.rightBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.HighGear ) ) );
      controller.leftBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.LowGear ) ) );
      
      
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
