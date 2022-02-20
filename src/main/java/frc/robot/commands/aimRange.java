// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Based on the Getting in Range and Aim Case Studies 
 */
public class aimRange extends CommandBase {

  private final DriveTrain m_drivetrain;
  private final LimeLight m_limelight;

  // Fields for range
  private boolean crosshairCalibrated = false;
  
  private double KpAngle;
  private double KpMeter;

  private double angleTolerance;
  private double meterTolerance;
  
  private double desiredDistance;

  private double driveAdjust; 
  private double distanceError;

  private double minDriveSpeed;

  // Fields for aim
  private double KpAim;

  private double aimTolerance;

  private double turnAdjust;
  private double headingError;
  
  private double minTurnSpeed;

  /**
   * Use this constructor cross-hair has been calibrated for the distance we want, 
   * thus we can use ty to command our distance error
   * 
   * @param drivetrain
   * @param limelight
   */
  public aimRange(DriveTrain drivetrain, LimeLight limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;

    this.desiredDistance = 0;
    crosshairCalibrated = true;
    
    addRequirements(drivetrain, limelight);
  }

  /**
   * Use this constructor if we have not calibrated for the distance we want, or for whatever else reason their might be.
   * Will use our current distance to adjust our error.
   * 
   * @param drivetrain
   * @param limelight
   * @param desiredDistance
   */
  public aimRange(DriveTrain drivetrain, LimeLight limelight, double desiredDistance) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;

    this.desiredDistance = desiredDistance;
    crosshairCalibrated = false;
    
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize Range Fields
    KpAngle = 0.005;
    KpMeter = 0.005;

    angleTolerance = 0.1;
    meterTolerance = 0.1;
    
    minDriveSpeed = 0.345;

    // Initialize Aim Fields
    KpAim = 0.03;
    minTurnSpeed = 0.05;
    aimTolerance = 0.8;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizontalDegTarget = m_limelight.getHorizontalDegToTarget(); 

    headingError = ((horizontalDegTarget != 0) ? -horizontalDegTarget : headingError);

    if(!crosshairCalibrated){
        double currentDistance = m_limelight.estimateDistance(0); // Input actually height from target later
        
        distanceError = desiredDistance - currentDistance;
        driveAdjust = KpMeter * distanceError;
    }
    else{
        double verticalDegTarget = m_limelight.getVerticalDegToTarget();

        distanceError = ((verticalDegTarget != 0) ? -verticalDegTarget : distanceError);
        driveAdjust = KpAngle * distanceError;
    }

    driveAdjust = ((Math.abs(driveAdjust) < minDriveSpeed) ? minDriveSpeed : driveAdjust);
    driveAdjust = ((distanceError > 0) ? -driveAdjust: driveAdjust);

    turnAdjust = KpAim * headingError; // Multiplies our error by our speed constant, that way we have a useable speed
    turnAdjust = ((Math.abs(turnAdjust) < minTurnSpeed) ? minTurnSpeed : turnAdjust); // Ensures we do go under min speed needed to turn
    turnAdjust = ((headingError > 0) ? -turnAdjust : turnAdjust); // Ensures correct directional change

    m_drivetrain.arcadeDrive(driveAdjust, turnAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(Math.abs(headingError) <= aimTolerance){
        if(!crosshairCalibrated){
            return Math.abs(distanceError) <= meterTolerance; 
          }
          else{
            return Math.abs(distanceError) <= angleTolerance;
          }
      }
      else{
          return false;
      }
        
  }
}
