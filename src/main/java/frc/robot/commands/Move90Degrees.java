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

public class Move90Degrees extends Command {

  private OI oi;
  private DriveTrain driveTrain;
  private static double currentAngle;
  private static double increment = 90;
  private static final double tolerance = 5;
  

  public Move90Degrees() {
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
      if(currentAngle > 0 && currentAngle < 90) {
        if(increment < Robot.getCurrentAngle()+tolerance && increment > Robot.getCurrentAngle()-tolerance) {
            driveTrain.drive(0, 0);
        } else {
            driveTrain.drive(-0.5, 0.5);
        }
      } else if(currentAngle > 90 && currentAngle < 180) {
        if(increment*2 < Robot.getCurrentAngle()+tolerance && increment*2 > Robot.getCurrentAngle()-tolerance) {
            driveTrain.drive(0, 0);
        } else {
            driveTrain.drive(-0.5, 0.5);
        }
      } else if(currentAngle > 180 && currentAngle < 270) {
        if(increment*3 < Robot.getCurrentAngle()+tolerance && increment*3 > Robot.getCurrentAngle()-tolerance) {
            driveTrain.drive(0, 0);
        } else {
            driveTrain.drive(-0.5, 0.5);
        }
      } else if(currentAngle > 270 && currentAngle < 360) {
        if(increment*4 < Robot.getCurrentAngle()+tolerance && increment*4 > Robot.getCurrentAngle()-tolerance) {
            driveTrain.drive(0, 0);
        } else {
            driveTrain.drive(-0.5, 0.5);
        }
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