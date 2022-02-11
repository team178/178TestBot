// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import libs.IO.ConsoleController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.aimingTest;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_drivetrain = new DriveTrain();
  private final LimeLight m_limelight = new LimeLight();

  //Creates joystick object for the Main and Aux controllers
  private final ConsoleController m_controller = new ConsoleController(0);
  private final Joystick m_jJoystick = new Joystick(0);

  //USB Camera declarations
  private final UsbCamera camera1;
  private final UsbCamera camera2;

  // Create SmartDashboard chooser for autonomous routines and drive
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_driveChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

     // Sets driving to either the main joystick or aux xbox controller (Note to self make method in Joysticks that adjust for thrust and twist)
    
     //m_drivetrain.setDefaultCommand(
        //new TankDrive(m_joystick.getLeftY(), m_joystick.getRightY(), m_drivetrain));
    //m_drivetrain.setDefaultCommand(
        //new TankDrive(m_joystick::getLeftStickY, m_joystick::getRightStickY, m_drivetrain));

    if(RobotBase.isReal()){
      //Camera 1
      camera1 = CameraServer.startAutomaticCapture("cam0", 0);
      //camera1.setResolution(160, 90);
      camera1.setFPS(14);
      camera1.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras

      //Camera 2
      camera2 = CameraServer.startAutomaticCapture("cam1", 1);
      //camera2.setResolution(160, 120);
      camera2.setFPS(14);
      camera2.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
    }
    else{
      camera1 = null;
      camera2 = null;
    }

    // Configure the button bindings
    configureButtonBindings();
    configureShuffleBoard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  private void configureShuffleBoard() {

    //Drive Routine Options (How our robot is going to drive)
    m_driveChooser.setDefaultOption("Controller Tank Drive", new TankDrive(m_controller::getLeftStickY, m_controller::getRightStickY, m_drivetrain));
    m_driveChooser.addOption("Controller Arcade Drive", new ArcadeDrive(m_controller::getLeftStickY, m_controller::getRightStickX, m_drivetrain));
    
    m_driveChooser.addOption("Joystick Tank Drive", new TankDrive(m_jJoystick::getX, m_jJoystick::getY, m_drivetrain));
    m_driveChooser.addOption("Joystick Arcade Drive", new ArcadeDrive(m_jJoystick::getY, m_jJoystick::getTwist, m_drivetrain));

    //Autonomous Chooser Options (How our robot is going to tackle auto)
    m_autoChooser.setDefaultOption("Aiming Using Vision", new aimingTest(m_drivetrain, m_limelight));

    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");

    driveBaseTab
      .add("Autonomous Routine", m_autoChooser)
        .withSize(2, 1);

    driveBaseTab
      .add("Drive Routine", m_driveChooser)
        .withSize(2, 1);

    OIConstants.kDriveSpeedMult = driveBaseTab
    .add("Max Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 2)) // specify widget properties here
          .getEntry();

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

  public Command getDriveCommand(){
    return m_driveChooser.getSelected();
  }
}