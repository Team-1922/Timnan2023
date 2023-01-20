// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.KeyPoint;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class DriveTrainSubsystem extends SubsystemBase { 
  private CANSparkMax m_leftLead = new CANSparkMax(Constants.kLeftLead, MotorType.kBrushless);
  private CANSparkMax m_leftFollow = new CANSparkMax(Constants.kLeftFollow, MotorType.kBrushless);
  private CANSparkMax m_rightLead = new CANSparkMax(Constants.kRightLead, MotorType.kBrushless);
  private CANSparkMax m_rightFollow = new CANSparkMax(Constants.kRightFollow, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerLeft;
  private SparkMaxPIDController m_pidControllerRight;
  double kp;
  double ki;
  double kd;
 double kff;
 double kiz;
 double maxrpm;
 double rightkp, rightki, rightkd, rightkff, rightkiz, rightmaxrpm;
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {


    SmartDashboard.putNumber("left p gain", kp);
    SmartDashboard.putNumber("left i gain", ki);
   //
    SmartDashboard.putNumber("left d gain", kd); /* */ 
    SmartDashboard.putNumber("left feed foward ", kff);
    SmartDashboard.putNumber("left i zone", kiz);
    SmartDashboard.putNumber("left max rpm", maxrpm);


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
 //m_pidControllerLeft.setRefrence();
  //m_pidControllerRight.setRefrence();
  m_pidControllerLeft.setP(SmartDashboard.getNumber( "left p gain" , 0));
  m_pidControllerLeft.setI(ki);
  m_pidControllerLeft.setD(kd);
 m_pidControllerLeft.setOutputRange(0, 1); //change that later, 1 and 0 are placeholders
 m_pidControllerLeft.setFF(kff); //feed foward
m_pidControllerLeft.setIZone(kiz); //i zone

double leftsetpoint = RobotContainer.LeftJoystick.getY()*maxrpm;
m_pidControllerLeft.setReference(leftsetpoint, CANSparkMax.ControlType.kVelocity);

     // right side
m_pidControllerRight.setP(rightkp);

m_pidControllerRight.setI(rightki);
m_pidControllerRight.setD(rightkd);
m_pidControllerRight.setOutputRange(0, 1);
m_pidControllerRight.setFF(rightkff);
m_pidControllerRight.setIZone(rightkiz);
double rightsetpoint = RobotContainer.RightJoystick.getY()*maxrpm;
m_pidControllerRight.setReference(rightsetpoint, CANSparkMax.ControlType.kVelocity);



 /* double MaxVelocity = 250; // the max velocity of the motor , test this when the drivebase is done
 double OutputScale = .9 ; //scale the output 
Drive( RobotContainer.LeftJoystick.getY()*MaxVelocity*OutputScale, RobotContainer.RightJoystick.getY()*MaxVelocity*OutputScale); */
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  

 //
//SmartDashboard.getNumber(getName(), kd)


  }
}
