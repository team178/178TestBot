/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {

  private final SPI.Port sPort = SPI.Port.kOnboardCS0;

  private final MotorController m_leftMotor =
    new MotorControllerGroup(new PWMTalonSRX(RobotMap.DMTopLeft), new PWMVictorSPX(RobotMap.DMBottomLeft));
  
  private final MotorController m_rightMotor = 
    new MotorControllerGroup(new PWMTalonSRX(RobotMap.DMTopRight), new PWMVictorSPX(RobotMap.DMBottomRight));
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(sPort);
  private final Encoder m_leftEncoder = new Encoder(1, 2); //Will need actual digital input channels later
  private final Encoder m_rightEncoder = new Encoder(3, 4); //Will need actual digital input channels later
  
  /** Create a new drivetrain subsystem. */
  public DriveTrain() {
    super();

    m_leftMotor.setInverted(true);

    m_leftEncoder.setDistancePerPulse((Units.inchesToMeters(6) * Math.PI) / 1024); //1024 is what I guessed for EncoderTicks
    m_rightEncoder.setDistancePerPulse((Units.inchesToMeters(6) * Math.PI) / 1024); //1024 is what I guessed for EncoderTicks

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
   * @return The robots heading in degrees.
   */
  public double getAngle() {
    return -m_gyro.getAngle();
  }

    /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  public double getDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
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
