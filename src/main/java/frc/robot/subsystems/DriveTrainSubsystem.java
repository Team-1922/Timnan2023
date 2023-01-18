// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class DriveTrainSubsystem extends SubsystemBase { 
  private CANSparkMax m_leftLead = new CANSparkMax(Constants.leftLead, MotorType.kBrushless);
  private CANSparkMax m_leftFollow = new CANSparkMax(Constants.leftFollow, MotorType.kBrushless);
  private CANSparkMax m_rightLead = new CANSparkMax(Constants.rightLead, MotorType.kBrushless);
  private CANSparkMax m_rightFollow = new CANSparkMax(Constants.rightFollow, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerLeft;
  private SparkMaxPIDController m_pidControllerRight;
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    //Motor controlers
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);
    m_pidControllerLeft = m_leftLead.getPIDController();
    m_pidControllerRight = m_rightLead.getPIDController();
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
  

  }
}
