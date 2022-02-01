// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.Calibrate;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GyroButton;
import frc.robot.commands.Move90Degrees;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import libs.OI.Joysticks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TankDrive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveTrain m_drivetrain = new DriveTrain();

  //Creates joystick object for the Main and Aux controllers
  private final Joysticks m_joystick = new Joysticks();

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

     // Sets driving to either the main joystick or aux xbox controller (Note to self make method in Joysticks that adjust for thrust and twist)
    
     //m_drivetrain.setDefaultCommand(
        //new TankDrive(m_joystick.getLeftY(), m_joystick.getRightY(), m_drivetrain));
    m_drivetrain.setDefaultCommand(
        new TankDrive(m_joystick::getLeftStickYAux, m_joystick::getRightStickYAux, m_drivetrain));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    m_joystick.rightPadBottom1
        .whenPressed(new GyroButton(180));
    m_joystick.rightPadBottom2
        .whenPressed(new Calibrate());
    m_joystick.rightPadBottom3
        .whenPressed(new Move90Degrees());

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Example Command", m_autoCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}