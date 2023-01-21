// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrainSubsystem extends SubsystemBase { 
  private CANSparkMax m_leftLead = new CANSparkMax(Constants.kLeftLead, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder;
  private CANSparkMax m_leftFollow = new CANSparkMax(Constants.kLeftFollow, MotorType.kBrushless);
  private CANSparkMax m_rightLead = new CANSparkMax(Constants.kRightLead, MotorType.kBrushless);
  private RelativeEncoder m_rightEncoder;
  private CANSparkMax m_rightFollow = new CANSparkMax(Constants.kRightFollow, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerLeft;
  private SparkMaxPIDController m_pidControllerRight;
  private AHRS m_navX;

  private DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(AHRS navX) {
    //Motor controlers
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_leftEncoder = m_leftLead.getEncoder();

    m_rightLead.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();
    m_rightEncoder = m_rightLead.getEncoder();

    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);

    m_pidControllerLeft = m_leftLead.getPIDController();
    m_pidControllerRight = m_rightLead.getPIDController();

    m_navX = navX;

    // Setting up the odometry object in the constructor--A little sketchy? No errors and it builds though
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(robotPitch()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));
  }
  
public void Drive(double leftSpeed, double rightSpeed){
  m_leftLead.set(leftSpeed);
  m_rightLead.set(rightSpeed);

}
public void velocityDrive(double velocity){
//  m_pidControllerLeft.setRefrence();
//  m_pidControllerRight.setRefrence();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(robotPitch()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));
  

  }


  // Returns the navX Yaw, it's up and down like the way your neck moves 
  public double robotYaw(){
    return m_navX.getYaw();
  }
  // Returns the navX Pitch, it's side to side like the way a turntable rotates
  public double robotPitch(){
    return m_navX.getPitch();
  }

  

  public double getLeftEncoderRaw(){
    return m_leftEncoder.getPosition();
  }
  public double getLeftEncoderFeet(){
    return m_leftEncoder.getPosition() * Constants.EncoderTicksToFeet;
  }

  public double getRightEncoderRaw(){
    return m_rightEncoder.getPosition();
  }
  public double getRightEncoderFeet(){
    return m_rightEncoder.getPosition() * Constants.EncoderTicksToFeet;
  }


}
