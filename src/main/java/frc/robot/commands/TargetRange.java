/*package frc.robot.commands;

public class TargetRange {
    
}*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/** Aim towards vision target */
public class TargetRange extends CommandBase {

    private final PhotonCamera m_camera;
    private final DriveTrain m_drivetrain;
    private final PIDController m_turnController;

    /**
     * Creates a new TargetAim command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TargetRange(PhotonCamera camera, DriveTrain drivetrain) {
        m_camera = camera;
        m_drivetrain = drivetrain;
        m_turnController = new PIDController(Constants.DriveConstants.kTurnP.getDouble(0), 0, 0);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        //IDK if lines 55-62 are supposed to go in this section...

         // Constants such as camera and target height stored. Change per robot and goal!
        final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

        // Angle between horizontal and the camera.
        final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
        // How far from the target we want to be
        final double GOAL_RANGE_METERS = Units.feetToMeters(3);


        //double rotationSpeed;
        double forwardSpeed;
        double rotationSpeed = 0.0;

        // Apparently you can use 'var' in Java 10 and up and it'll get the return type
        // based off the function, class, or value.
        // Will probably change this later but would be interesting if it works
        var result = m_camera.getLatestResult();

       
       
        if (result.hasTargets()) {
            double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -m_turnController.calculate(range, GOAL_RANGE_METERS);
            //rotationSpeed = -m_turnController.calculate(result.getBestTarget().getYaw(), 0);
            //rotationSpeed = rotationSpeed < .365 && rotationSpeed > 0 ? .365 : rotationSpeed;
        } else {
            forwardSpeed = 0;
        }
        
        m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
