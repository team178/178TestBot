// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 

    https://www.chiefdelphi.com/t/help-with-limelight-code/352890/21 - Looked through this forum 
    
    Also it's important to note that we do not want oscallation, that means our kP is too low
    
    "Beware, if you set Kp or min_command too high, your robot can become unstable and can oscillate back and forth as it overshoots the target"
    
*/

public class modifiedAim extends CommandBase {

  private final DriveTrain m_drivetrain;
  private final LimeLight m_limelight;
  
  private double Kp = 0.03; // Speed Constant, should ensure we're not overshooting and getting out of range
  private double min_command = 0.05; // Min Command ensures our low kP doesn't make out robot stop as it gets closer to the tolerance (allows the robot keep moving)
  private double tolerance = 1; // Once it reads that limelight is within the tolerance, command ends
  
  private double tx;
  private double steering_adjust; 
  private double heading_error;

  private double m_left;
  private double m_right; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public modifiedAim(DriveTrain drivetrain, LimeLight limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = m_limelight.getHorizontalDegToTarget();
    heading_error = tolerance; // Previously this was negative (adjust if robot is not turning correctly)
    
    if (tx > tolerance){
          steering_adjust = Kp*heading_error + min_command;
    }
    else if (tx < -tolerance){
      steering_adjust = Kp*heading_error - min_command;
    }
      
    m_drivetrain.arcadeDrive(0, steering_adjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(heading_error) <= tolerance;
  }
}
