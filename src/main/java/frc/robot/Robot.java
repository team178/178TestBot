/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import frc.robot.commands.GyroButton;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static DriveTrain driveTrain;
  public static OI oi;
  private static double currentAngle;
  //private static final double smallTolerance = .1;

  //USB Camera declarations
  public static CameraServer cameraServer;
  public static UsbCamera camera1;
  public static UsbCamera camera2;
  
  public static SendableChooser<String> course = new SendableChooser<>();
  
  /*
   * Called regardless of mode
  */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    oi = new OI();

    //Camera initializations
    cameraServer = CameraServer.getInstance();
    
    //Camera 1
    camera1 = cameraServer.startAutomaticCapture("cam0", 0);
    //camera1.setResolution(160, 90);
    camera1.setFPS(14);
    camera1.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras

    //Camera 2
    camera2 = CameraServer.getInstance().startAutomaticCapture("cam1", 1);
    //camera2.setResolution(160, 120);
    camera2.setFPS(14);
    camera2.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras

    driveTrain.reset();
  }

  @Override
  public void robotPeriodic() {
    System.out.println("Gyro Reading: " + driveTrain.getAngle());
    
    if(driveTrain.getAngle()%360 == 0)
    {
      currentAngle = driveTrain.getAngle();
    } else {
      currentAngle = Math.abs(driveTrain.getAngle()%360);
    }
    System.out.println("Current Angle Rading: " + currentAngle);
  }

  /*
   * Called when robot is on but disabled
  */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  /*
   * Called during autonomous mode
  */
  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /*
   * Called during driver controlled period
  */
  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    //System.out.println("yeet");
    Scheduler.getInstance().run();
  }
  
  /*
   * Called during test mode
  */
  @Override
  public void testInit() {
    
  }
  
  @Override
  public void testPeriodic() {

  }

  public static double getCurrentAngle() {
     return Robot.currentAngle;
  }
}
