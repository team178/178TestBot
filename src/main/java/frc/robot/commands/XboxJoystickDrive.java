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

public class XboxJoystickDrive extends Command {

  private OI oi;
  private DriveTrain driveTrain;

  private double lThrust;
  private double rThrust;

  private double thrustSpeed = 1.5;

  public XboxJoystickDrive() {
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
    // yReduction = oi.trigger.get() ? 0.5 : 1;
    // twistReduction = oi.trigger.get() ? 0.5 : 0.5;
    
    // //Determine drive values
    // yVal = oi.getLeftStickYAux() * yReduction;
    // twistVal = oi.getRightStickXAux() * twistReduction;
    // //Apply drive values
    // if(Math.abs(yVal) > 0.2 || Math.abs(twistVal) > 0.2) { 
    //   driveTrain.drive(yVal+twistVal, yVal-twistVal);
    // } else {
    //   driveTrain.drive(0,0);
    // }

    lThrust = oi.getLeftStickYAux() * thrustSpeed;
    rThrust = oi.getRightStickYAux() * thrustSpeed;

    if(Math.abs(lThrust) < 0.2) {
      lThrust = 0;
    }

    if(Math.abs(rThrust) < 0.2) {
      rThrust = 0;
    }

    driveTrain.drive(lThrust, rThrust);
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