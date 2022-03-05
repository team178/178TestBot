package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class limelightGroupCommand extends SequentialCommandGroup {
    
    public limelightGroupCommand(DriveTrain drivetrain, LimeLight limelight){
        addCommands(
            new modifiedRange(drivetrain, limelight, 4),
            new modifiedAim(drivetrain, limelight)
        );
    }
    
}
