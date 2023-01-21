package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    //! OLD CONSTANTS
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    // public static final int kEncoderCPR = 4096;
    // public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    // public static final double kEncoderDistancePerPulse =
    //     // Assumes the encoders are directly mounted on the wheel shafts
    //     (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static NetworkTableEntry kTurnP;
    public static NetworkTableEntry kTurnI;
    public static NetworkTableEntry kTurnD;

    public static NetworkTableEntry kDriveP;
    public static NetworkTableEntry kDriveI;
    public static NetworkTableEntry kDriveD;

    public static NetworkTableEntry kMinTurnSpeed;
    public static NetworkTableEntry kMinDriveSpeed;

    //* NEW CONSTANTS
  
    public static final int kL1MotorPort = 1;
    public static final int kL2MotorPort = 2;
    public static final int kR1MotorPort = 3;
    public static final int kR2MotorPort = 4;

    public static final int kEncoderCPR = 4096; // MagCoder constant
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerRev = 2 * (kWheelDiameter / 2) * Math.PI;
    public static final double kGearboxRatio = 1;
    // Gearbox ratio 1:1 because mag encoders attached to output shaft whereas falcons are before the gearbox

    public static final double kTrackWidth = Units.inchesToMeters(22);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final double kS = 0.91843; //! Need to be tuned
    public static final double kV = 33.753;
    public static final double kA = 6.124;

    public static final double kPVel = 5.1978;

  }
  
  public static final class OIConstants {
    public static final int kJoystickPort = 0;
    public static final int kControllerPort = 1;

    public static NetworkTableEntry kDriveSpeedMult1;
    public static NetworkTableEntry kDriveSpeedMult2;
  }
}
