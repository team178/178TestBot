/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  
  //DM declarations
  public static VictorSPX left1;
  public static VictorSPX left2;
  public static VictorSPX right1;
  public static VictorSPX right2;

  private final Gyro gyro = new ADXRS450_Gyro(sPort);
    
  public DriveTrain() {
	  //Init DMs
	  left1 = new VictorSPX(RobotMap.DMTopLeft);
	  left2 = new VictorSPX(RobotMap.DMBottomLeft);
	  right1 = new VictorSPX(RobotMap.DMTopRight);
	  right2 = new VictorSPX(RobotMap.DMBottomRight);
	  
	  //Set victors to slaves
	  left2.follow(left1);
	  right2.follow(right1);
	  
	  //Config left motor & sensor directions
	  left1.setInverted(true);
	  left1.setSensorPhase(true);
	  left2.setInverted(InvertType.FollowMaster);
	  
	  //Config right motor & sensor directions
	  right1.setInverted(false);
	  right1.setSensorPhase(false);
	  right2.setInverted(InvertType.FollowMaster);
  }
  
  public void drive(double leftPower, double rightPower) {
    left1.set(ControlMode.PercentOutput, leftPower);
    right1.set(ControlMode.PercentOutput, rightPower);
  }

  public void calibrate() {
    gyro.calibrate();
  }

  public void reset() {
    gyro.reset();
  }

  public double getAngle() {
    return -gyro.getAngle();
  }

  @Override
  public void periodic() {
    setDefaultCommand(new JoystickDrive());
  }

  @Override
  public void initDefaultCommand()
  {
    setDefaultCommand(new JoystickDrive());
  }
}
