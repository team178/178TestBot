// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class aimingTest extends CommandBase {

  private final DriveTrain m_drivetrain;
  private final LimeLight m_limelight;
  
  private double Kp = 0.1;
  private double min_command = 0.05;
  private double tolerance = 0.1;
  
  public double tx;
  private double steering_adjust; 
  private double heading_error;

  private double m_left;
  private double m_right; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public aimingTest(DriveTrain drivetrain, LimeLight limelight) {
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
    heading_error = -tx;
    
    if (tx > 1.0){
          steering_adjust = Kp*heading_error - min_command;
          //adjusts horizontal change aka drive train turning
         
    }
    else if (tx < 1.0){
      steering_adjust = Kp*heading_error + min_command;
     
    }
      System.out.println(tx);
    m_left += steering_adjust;
    m_right -= steering_adjust;

    m_drivetrain.tankDrive(m_left * 0.5, m_right * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(heading_error) <= tolerance;
  }
}
