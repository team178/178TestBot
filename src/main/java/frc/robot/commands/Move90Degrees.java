/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;


public class Move90Degrees extends Command {

  private DriveTrain m_drivetrain;

  //private static double currentAngle;
  private static double increment = 90;
  private static final double tolerance = 5;
  

  public Move90Degrees(DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    requires(drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if(m_drivetrain.getAngle() > 270 && m_drivetrain.getAngle() < 360) {
        if(increment*4 < m_drivetrain.getAngle()+tolerance && increment > m_drivetrain.getAngle()-tolerance) {
          m_drivetrain.drive(0, 0);
        } else {
          m_drivetrain.drive(-0.5, 0.5);
        }
      } else if(m_drivetrain.getAngle() > 180 && m_drivetrain.getAngle() < 270) {
        if(increment*3 < m_drivetrain.getAngle()+tolerance && increment*3 > m_drivetrain.getAngle()-tolerance) {
          m_drivetrain.drive(0, 0);
        } else {
          m_drivetrain.drive(-0.5, 0.5);
        }
      } else if(m_drivetrain.getAngle() > 90 && m_drivetrain.getAngle() < 180) {
        if(increment*2 < m_drivetrain.getAngle()+tolerance && increment*2 > m_drivetrain.getAngle()-tolerance) {
          m_drivetrain.drive(0, 0);
        } else {
          m_drivetrain.drive(-0.5, 0.5);
        }
      } else if(m_drivetrain.getAngle() > 0 && m_drivetrain.getAngle() < 90) {
        if(increment < m_drivetrain.getAngle()+tolerance && increment > m_drivetrain.getAngle()-tolerance) {
          m_drivetrain.drive(0, 0);
        } else {
          m_drivetrain.drive(-0.5, 0.5);
        }
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if( (m_drivetrain.getAngle() > 70 && m_drivetrain.getAngle() < 90)||(m_drivetrain.getAngle() > 160 && m_drivetrain.getAngle() < 180)||
    (m_drivetrain.getAngle() > 250 && m_drivetrain.getAngle() < 270)||(m_drivetrain.getAngle() > 340 && m_drivetrain.getAngle() < 360) ) {
      return true;
    } else {
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

}