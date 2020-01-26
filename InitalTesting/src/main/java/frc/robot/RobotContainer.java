/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainTalon drive;
  public final OneWheelShooter shoot;
  public final ControlPanel control;

  private final Command m_autoCommand = null;   
  
  public final DigitalInput limit;
  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry tx, ty, targetVisible;
  
  double angleLimelight = 0;
  double heightLimelight = 0;
  double heightTarget = 0;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new DrivetrainTalon();
    limit = new DigitalInput(0);
    shoot = new OneWheelShooter();
    control = new ControlPanel();
    
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    targetVisible = table.getEntry("target");


    // Configure the button bindings
    configureButtonBindings();
  }

  public double getDistanceToTarget(){
      return targetVisible.getBoolean( false ) ? ( heightTarget - heightLimelight ) / Math.tan( angleLimelight + ty.getDouble( 0 ) ) : 0;
  }

  public double getAngleToTarget(){
      return targetVisible.getBoolean( false ) ? tx.getDouble( 0 ) : 0;
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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
