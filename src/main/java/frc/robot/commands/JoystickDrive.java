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

public class JoystickDrive extends Command {

  private OI oi;
  private DriveTrain driveTrain;
  
  private double yVal;
  private double twistVal;
  private double yReduction;
  private double twistReduction;

  public JoystickDrive() {
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
    //Determine reductions
    yReduction = oi.trigger.get() ? 0.5 : 1;
    twistReduction = oi.trigger.get() ? 0.4 : 1;
    
    //Determine drive values
    yVal = oi.getY() * yReduction;
    twistVal = oi.getTwist() * twistReduction;
    //Apply drive values
    if(Math.abs(yVal) > 0.1 || Math.abs(twistVal) > 0.1) { 
      driveTrain.drive(yVal+twistVal, yVal-twistVal);
    } else {
      driveTrain.drive(0,0);
    }
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
