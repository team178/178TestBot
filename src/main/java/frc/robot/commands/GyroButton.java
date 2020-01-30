/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class GyroButton extends Command {

  private OI oi;
  private DriveTrain driveTrain;
  private static double currentAngle;
  private static double desiredAngle = 180;
  private static final double tolerance = 15;
  private static final double smallTolerance = .1;

  public GyroButton() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    oi = Robot.oi;
    driveTrain = Robot.driveTrain;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  

    if(driveTrain.getAngle()-360 <= Math.abs(smallTolerance))
    {
      currentAngle = 0; 
    } else {
      currentAngle = driveTrain.getAngle();
    }
    if(desiredAngle < driveTrain.getAngle()+tolerance && desiredAngle > driveTrain.getAngle()-tolerance)
    {
      driveTrain.drive(0, 0);
    }else{
      driveTrain.drive(-0.5, 0.5);
    }
    System.out.println(currentAngle);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(desiredAngle < driveTrain.getAngle()+tolerance && desiredAngle > driveTrain.getAngle()-tolerance)
    {
      return true;
    }else{
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  public static double printAngle() {
    return GyroButton.currentAngle;
  }

}