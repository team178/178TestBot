// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;

public class Drivetrain extends SubsystemBase {
  
  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(sPort);
    
  private WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(DriveConstants.kL1MotorPort);
  private WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(DriveConstants.kL2MotorPort);

  private WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(DriveConstants.kR1MotorPort);
  private WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(DriveConstants.kR2MotorPort);

  private DifferentialDrivePoseEstimator m_poseEstimator;

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kPVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kPVel, 0, 0);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  private final Field2d m_field = new Field2d();

  private final Matrix<N3, N1> stupidTrustMatrix;

  /* Creates a new Drivetrain. */
  public Drivetrain() {

    m_leftMotor.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    m_rightMotor.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    m_leftFollower.follow(m_leftMotor);
    m_rightFollower.follow(m_rightMotor);

    m_leftMotor.setSensorPhase(false);
    m_rightMotor.setSensorPhase(false);

    //! Will need to be adjusted until we're going the right direction
    // Teleop drive input is inverted in arcadeDrive command to be compatible with autos
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_gyro.calibrate();

    m_poseEstimator = new DifferentialDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      getLeftEncoderPositionMeters(),
      getRightEncoderPositionMeters(),
      new Pose2d()
    );

    stupidTrustMatrix = new Matrix<N3, N1>(N3.instance, N1.instance);
    stupidTrustMatrix.set(0, 0, 3);
    stupidTrustMatrix.set(1, 0, 3);
    stupidTrustMatrix.set(2, 0, 3);

    m_poseEstimator.setVisionMeasurementStdDevs(stupidTrustMatrix);
  }

  public void resetGyro() {
    m_gyro.reset();
  }
  
  public void resetEncoders() {
    m_leftMotor.setSelectedSensorPosition(0);
    m_rightMotor.setSelectedSensorPosition(0);
  }

  public double getGyroHeading() {
    return m_gyro.getAngle();
  }

  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  public double getLeftEncoderPosition() {
    return m_leftMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderPositionMeters() {
    return talonUnitsToMeters(m_leftMotor.getSelectedSensorPosition());
  }
  
  public double getRightEncoderPosition() {
    return m_rightMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPositionMeters() {
    return talonUnitsToMeters(m_rightMotor.getSelectedSensorPosition());
  }

  public double getLeftEncoderVelocity() {
    return m_leftMotor.getSelectedSensorVelocity();
  }

  public double getLeftEncoderVelocityMeters() {
    return talonUnitsToMeters(m_leftMotor.getSelectedSensorVelocity());
  }
  
  public double getRightEncoderVelocity() {
    return m_rightMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocityMeters() {
    return talonUnitsToMeters(m_rightMotor.getSelectedSensorVelocity());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftEncoderVelocityMeters(),
      getRightEncoderVelocityMeters()
    );
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        getLeftEncoderPositionMeters(),
        getRightEncoderPositionMeters(),
        pose
      );
  }

  private double talonUnitsToMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / DriveConstants.kEncoderCPR;
    double wheelRotations = motorRotations / DriveConstants.kGearboxRatio;
    double positionMeters = wheelRotations * DriveConstants.kEncoderDistancePerRev;
    return positionMeters;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
  }

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setWheelSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
    
    if (leftSpeed > 0.15) {
      leftSpeed = 0.15;
    }
    if (rightSpeed > 0.15) {
      rightSpeed = 0.15;
    }

    final double leftFeedforward = m_feedforward.calculate(leftSpeed);
    final double rightFeedforward = m_feedforward.calculate(rightSpeed);

    final double leftOutput = m_leftPIDController.calculate(getLeftEncoderVelocityMeters(), leftSpeed);
    final double rightOutput = m_rightPIDController.calculate(getRightEncoderVelocityMeters(), rightSpeed);
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }
  
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rot, double deadzone) {
    return this.run(() -> {
      arcadeDrive(
          MathUtil.applyDeadband(forward.getAsDouble(), deadzone) * 0.1,
          MathUtil.applyDeadband(rot.getAsDouble(), deadzone) * 0.3
          );
    }).repeatedly();
  }
  
  public void arcadeDrive(double forward, double rot) {
    var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(
      new ChassisSpeeds(-forward, 0.0, -rot)
    );
    setWheelSpeeds(wheelSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_poseEstimator.update(getGyroRotation(), getRightEncoderPositionMeters(), getRightEncoderPositionMeters());
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    if(table.containsKey("botpose")) {
      double[] botposeEntry = table.getEntry("botpose").getDoubleArray(new double[1]);
      
      if (botposeEntry.length > 0) {
        // The pose from limelight for some reason has it's orign in the middle of the field instead
        // of the bottom left like the WPILib pose estimator, so we have to account for that
        Pose2d botpose = new Pose2d(
            botposeEntry[0] + FieldConstants.kFieldLength / 2,
            botposeEntry[1] +  FieldConstants.kFieldWidth / 2,
            Rotation2d.fromDegrees(botposeEntry[5])
        );

        m_poseEstimator.addVisionMeasurement(
            botpose,
            Timer.getFPGATimestamp(),
            stupidTrustMatrix
        );
      }
      
    }
    

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putData(m_field);

    SmartDashboard.putNumber("LEncoderPos", getLeftEncoderPosition());
    SmartDashboard.putNumber("REncoderPos", getRightEncoderPosition());

    SmartDashboard.putNumber("LEncoderM", getLeftEncoderPositionMeters());
    SmartDashboard.putNumber("REncoderM", getRightEncoderPositionMeters());

    SmartDashboard.putData(m_gyro);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
