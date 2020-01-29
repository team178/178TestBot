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
  private static double angle = 180;
  private static final double tolerance = 15;

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

   //angle = driveTrain.getAngle();
   //angle = angle - 180;
   driveTrain.drive(0.5, -0.5);

   if(angle < driveTrain.getAngle()+tolerance && angle > driveTrain.getAngle())
   {
       driveTrain.drive(0, 0);
   }

   /*while(angle >= driveTrain.getAngle()+tolerance || angle <= driveTrain.getAngle()-tolerance)
   {
     driveTrain.drive(0.5,-0.5);
     //System.out.println("YEET");
   }
   System.out.println("NOT YEETING");
   driveTrain.drive(0, 0);*/
   
/*if(angle <= driveTrain.getAngle()+tolerance && angle >= driveTrain.getAngle()-tolerance)
{
    driveTrain.drive(0,0);
}*/
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
}
