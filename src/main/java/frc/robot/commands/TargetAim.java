// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/** Aim towards vision target */
public class TargetAim extends CommandBase {

    private final PhotonCamera m_camera;
    private final DriveTrain m_drivetrain;
    private final PIDController m_turnController;

    /**
     * Creates a new TargetAim command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TargetAim(PhotonCamera camera, DriveTrain drivetrain) {
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

        double rotationSpeed;

        // Apparently you can use 'var' in Java 10 and up and it'll get the return type
        // based off the function, class, or value.
        // Will probably change this later but would be interesting if it works
        var result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            rotationSpeed = -m_turnController.calculate(result.getBestTarget().getYaw(), 0);
            rotationSpeed = rotationSpeed < .365 && rotationSpeed > 0 ? .365 : rotationSpeed;
        } else {
            rotationSpeed = 0;
        }
        System.out.println(rotationSpeed);
        m_drivetrain.tankDrive(rotationSpeed, -rotationSpeed);
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
