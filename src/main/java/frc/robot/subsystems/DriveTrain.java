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
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.DriveConstants;


/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {

  private final SPI.Port sPort = SPI.Port.kOnboardCS0;

  private final MotorController m_leftMotor =
    new MotorControllerGroup(new WPI_TalonSRX(DriveConstants.kLeftMotor1Port), new WPI_VictorSPX(DriveConstants.kLeftMotor2Port));
  
  private final MotorController m_rightMotor = 
    new MotorControllerGroup(new WPI_TalonSRX(DriveConstants.kRightMotor1Port), new WPI_VictorSPX(DriveConstants.kRightMotor2Port));
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(sPort);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);
  
  /** Create a new drivetrain subsystem. */
  public DriveTrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    addChild("Drive", m_drive);
    addChild("Gyro", m_gyro);
    addChild("Left Encoder", m_leftEncoder);
    addChild("Right Encoder", m_rightEncoder);
  }
  
  /**
   * Tank style driving for the Drivetrain.
   *
   * @param left Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void drive(double leftSpeed, double rightSpeed) {
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

  /** Calibrates the gyro */
  public void calibrate() {
    m_gyro.calibrate();
  }

  /** Reset the robots sensors to the zero states. */
  public void reset() {
    m_gyro.reset();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Get the robot's heading.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getAngle() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

    /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  public double getDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("Left Distance", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Distance", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Speed", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Speed", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  @Override
  public void periodic() {
    log();
  }

}
