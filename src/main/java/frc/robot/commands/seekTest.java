// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class seekTest extends CommandBase {

  private final DriveTrain m_drivetrain;
  private final LimeLight m_limelight;
  
  private double Kp = -0.1;
  private double tolerance = 0.1;

  private boolean tv;
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
  public seekTest(DriveTrain drivetrain, LimeLight limelight) {
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
    tv = m_limelight.isTargetFound();
    
    if (tv){
      // We don't see the target, seek for the target by spinning in place at a safe speed.
      steering_adjust = 0.3f;
    }
    else{
       // We do see the target, execute aiming code
      heading_error = tx;
      steering_adjust = Kp * tx;
    }
      
    m_left += steering_adjust;
    m_right -= steering_adjust;

    m_drivetrain.drive(m_left, m_right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(heading_error) <= tolerance;
  }
}
