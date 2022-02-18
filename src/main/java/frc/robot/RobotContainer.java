// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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
import frc.robot.commands.aimRangeTest;
import frc.robot.commands.aimingTest;
import frc.robot.commands.seekTest;
import frc.robot.commands.modifiedAim;


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
  private final Joystick m_joystick = new Joystick(0);

  //USB Camera declarations
  private final UsbCamera camera1;
  private final UsbCamera camera2;

  // Create SmartDashboard chooser for autonomous routines and drive
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_driveChooser = new SendableChooser<>();

  //Creates SmartDashboard chooser for drive axises (for example - Right Joystick Y controls Left side of the robot in TankDrive)
  private final SendableChooser<DoubleSupplier> m_driveAxis1 = new SendableChooser<>();
  private final SendableChooser<DoubleSupplier> m_driveAxis2 = new SendableChooser<>(); 

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
  private void configureButtonBindings() {
    m_controller.a.whenPressed(new aimingTest(m_drivetrain, m_limelight));
  }

  private void configureShuffleBoard() {
    //Drive Axis Control Options (example - LeftStickY)
    m_driveAxis1.setDefaultOption("Left Controller Stick Y", m_controller::getLeftStickY); 
      
      //For the Xbox Controller
      m_driveAxis1.addOption("Left Controller Stick X", m_controller::getLeftStickX);
      m_driveAxis1.addOption("Right Controller Stick X", m_controller::getRightStickX);
      m_driveAxis1.addOption("Right Controller Stick Y", m_controller::getRightStickY);
      m_driveAxis1.addOption("Left Controller Trigger", m_controller::getLeftTrigger);
      m_driveAxis1.addOption("Right Controller Trigger", m_controller::getRightTrigger);

      //For the Joystick
      m_driveAxis1.addOption("Joystick X", m_joystick::getX);
      m_driveAxis1.addOption("Joystick Y", m_joystick::getY);
      m_driveAxis1.addOption("Joystick Twist", m_joystick::getTwist);


    m_driveAxis2.setDefaultOption("Right Controller Stick Y", m_controller::getRightStickY);

      //For the Xbox Controller
      m_driveAxis2.addOption("Left Controller Stick X", m_controller::getLeftStickX);
      m_driveAxis2.addOption("Left Controller Stick Y", m_controller::getLeftStickY);
      m_driveAxis2.addOption("Right Controller Stick X", m_controller::getRightStickX);
      m_driveAxis2.addOption("Left Controller Trigger", m_controller::getLeftTrigger);
      m_driveAxis2.addOption("Right Controller Trigger", m_controller::getRightTrigger);

      //For the Joystick
      m_driveAxis2.addOption("Joystick X", m_joystick::getX);
      m_driveAxis2.addOption("Joystick Y", m_joystick::getY);
      m_driveAxis2.addOption("Joystick Twist", m_joystick::getTwist);

    //Drive Routine Options (How our robot is going to drive)
    m_driveChooser.setDefaultOption("Tank Drive", new TankDrive(m_driveAxis1, m_driveAxis2, m_drivetrain));
    m_driveChooser.addOption("Arcade Drive", new ArcadeDrive(m_driveAxis1, m_driveAxis2, m_drivetrain));

    //Autonomous Chooser Options (How our robot is going to tackle auto)
    m_autoChooser.setDefaultOption("Seeking using Vision", new seekTest(m_drivetrain, m_limelight));
    m_autoChooser.addOption("Aiming using Vision", new aimingTest(m_drivetrain, m_limelight));
    m_autoChooser.addOption("Aiming and Range Using Vision", new aimRangeTest(m_drivetrain, m_limelight));

    //Creates new Shuffleboard tab called Drivebase
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");

    //Adds a chooser to the Drivebase tab to select autonomous routine (before anything is ran)
    driveBaseTab
      .add("Autonomous Routine", m_autoChooser)
        .withSize(2, 1)
          .withPosition(0, 3);

    //Adds a chooser to the Drivebase tab to select drive routine (before anything is ran)
    driveBaseTab
      .add("Drive Routine", m_driveChooser)
        .withSize(2, 1)
          .withPosition(0, 0);
    
    //Adds a chooser to the Drivebase tab to select drive axis 1 (before anything is ran)
    driveBaseTab
      .add("Drive Axis 1", m_driveAxis1)
        .withSize(2, 1)
          .withPosition(2, 0);

    //Adds a chooser to the Drivebase tab to select drive axis 2 (before anything is ran)
    driveBaseTab
      .add("Drive Axis 2", m_driveAxis2)
        .withSize(2, 1)
          .withPosition(2, 3);
    
    //Adds a slider to the Drivebase tab so driver can adjust sensitivity for input 1 of the given drive command 
    OIConstants.kDriveSpeedMult1 = driveBaseTab
    .add("Max Speed for Joystick 1", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 2)) // specify widget properties here
          .withPosition(0, 1)
            .getEntry();
    
    //Adds a slider to the Drivebase tab so driver can adjust sensitivity for input 2 of the given drive command 
    OIConstants.kDriveSpeedMult2 = driveBaseTab
    .add("Max Speed for Joystick 2", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 2)) // specify widget properties here
          .withPosition(0, 2)
            .getEntry();

    //Adds a Layout (basically a empty list) to the Drivebase tab for Limelight Commands 
    ShuffleboardLayout limelightCommands = driveBaseTab
      .getLayout("Limelight Commands", BuiltInLayouts.kList)
        .withSize(2, 2)
          .withPosition(0, 4)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
    
    //Adds buttons to the aforementioned Layout that run Limelight related commands when selected
    limelightCommands.add(new seekTest(m_drivetrain, m_limelight));
    limelightCommands.add(new aimingTest(m_drivetrain, m_limelight));
    limelightCommands.add(new aimRangeTest(m_drivetrain, m_limelight));
    limelightCommands.add(new modifiedAim(m_drivetrain, m_limelight));

    //Adds a Layout (basically a empty list) to the Drivebase tab for Drive Commands which will allow drivers to change from TankDrive to Arcade drive (or any drive command) on the spot  
    ShuffleboardLayout driveCommands = driveBaseTab
      .getLayout("Drive Commands", BuiltInLayouts.kList)
        .withSize(2, 2)
          .withPosition(0, 6)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    //Adds buttons to the aforementioned Layout that run drive commands when selected
    driveCommands.add("Tank Drive", new TankDrive(m_driveAxis1, m_driveAxis2, m_drivetrain));
    driveCommands.add("Arcade Drive", new ArcadeDrive(m_driveAxis1, m_driveAxis2, m_drivetrain));
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
