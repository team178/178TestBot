/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.DriveConstants;


/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);

  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);

  private final MotorController m_leftMotor =
    new MotorControllerGroup(leftMaster, leftSlave);
  
  private final MotorController m_rightMotor = 
    new MotorControllerGroup(rightMaster, rightSlave);
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  /** Create a new drivetrain subsystem. */
  public DriveTrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(false);
    m_leftMotor.setInverted(true);

    // Sets the distance per pulse for the encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    
    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(false);

    addChild("Drive", m_drive);
  }
  
  /**
   * Tank style driving for the Drivetrain.
   *
   * @param left Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Reset the robots sensors to the zero states. */
  public void reset() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return rightMaster.getSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {}

}
