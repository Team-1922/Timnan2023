// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

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
  double kp, ki, kd, kff, kiz, kmaxrpm;
 double rightkp, rightki, rightkd, rightkff, rightkiz, rightmaxrpm;
 double kMinOutput, kMaxOutput;
double RightkMinOutput, RightkMaxOutput;



double p,i,d,ff,iz,Maxrpm,rightp,righti,rightd,rightff,rightiz,rightMaxrpm;
double minoutput, maxoutput, rightminoutput, rightmaxoutput;
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {


    SmartDashboard.putNumber("left p gain", kp);
    SmartDashboard.putNumber("left i gain", ki);
    SmartDashboard.putNumber("left d gain", kd); 
    SmartDashboard.putNumber("left feed foward ", kff);
    SmartDashboard.putNumber("left i zone", kiz);
    SmartDashboard.putNumber("left max rpm", kmaxrpm);
    SmartDashboard.putNumber("left max output", kMaxOutput);
    SmartDashboard.putNumber("left min output", kMinOutput);

    
    SmartDashboard.putNumber("right p gain", rightkp);
    SmartDashboard.putNumber("right i gain", rightki);
    SmartDashboard.putNumber("right d gain", rightkd); 
    SmartDashboard.putNumber("right feed foward ", rightkff);
    SmartDashboard.putNumber("right i zone", rightkiz);
    SmartDashboard.putNumber("right max rpm", rightmaxrpm);
    SmartDashboard.putNumber("right max output", RightkMaxOutput);
    SmartDashboard.putNumber("right min output", RightkMinOutput);

    //Motor controlers
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);
    m_pidControllerLeft = m_leftLead.getPIDController();
    m_pidControllerRight = m_rightLead.getPIDController();}


  private AHRS m_navX; 
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(AHRS navX) {
    //Motor controlers
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);
    m_pidControllerLeft = m_leftLead.getPIDController();
    m_pidControllerRight = m_rightLead.getPIDController();

    m_navX = navX;
  }
public void Drive(double leftSpeed, double rightSpeed){
  m_leftLead.set(leftSpeed);
  m_rightLead.set(rightSpeed);

}
public void velocityDrive(double velocity){

 //m_pidControllerLeft.setRefrence();
  //m_pidControllerRight.setRefrence();
  m_pidControllerLeft.setP(p);
  m_pidControllerLeft.setI(i);
  m_pidControllerLeft.setD(d);
 m_pidControllerLeft.setOutputRange(SmartDashboard.getNumber("left min output", 0), SmartDashboard.getNumber("left max output", 1)); 
 m_pidControllerLeft.setFF(ff); //feed foward
m_pidControllerLeft.setIZone(iz); //i zone

double leftsetpoint = RobotContainer.LeftJoystick.getY()*kmaxrpm;
m_pidControllerLeft.setReference(leftsetpoint, CANSparkMax.ControlType.kVelocity);

     // right side
m_pidControllerRight.setP(rightp);

m_pidControllerRight.setI(righti);
m_pidControllerRight.setD(rightd);
m_pidControllerRight.setOutputRange(rightminoutput, rightmaxoutput);
m_pidControllerRight.setFF(rightff);
m_pidControllerRight.setIZone(rightd);
double rightsetpoint = RobotContainer.RightJoystick.getY()*kmaxrpm;
m_pidControllerRight.setReference(rightsetpoint, CANSparkMax.ControlType.kVelocity);




 /* double MaxVelocity = 250; // the max velocity of the motor , test this when the drivebase is done
 double OutputScale = .9 ; //scale the output 
Drive( RobotContainer.LeftJoystick.getY()*MaxVelocity*OutputScale, RobotContainer.RightJoystick.getY()*MaxVelocity*OutputScale); */

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //left pid
  if (p != kp){  m_pidControllerLeft.setP(SmartDashboard.getNumber( "left p gain" , 0)); }
if (i != ki) {  m_pidControllerLeft.setI(SmartDashboard.getNumber("left i gain", 0)); }
 if (d != kd){ m_pidControllerLeft.setD(SmartDashboard.getNumber("left d gain", 0));}
  if (ff != kff) {m_pidControllerLeft.setFF(SmartDashboard.getNumber("left feed foward", 0)); } //feed foward
   if (iz != kiz) {m_pidControllerLeft.setIZone(SmartDashboard.getNumber("left i zone", 0));}
if (minoutput != kMinOutput){minoutput = kMinOutput;}
if (maxoutput != kMaxOutput){maxoutput = kMaxOutput;}


   // right pid
    if (rightp != rightkp){ m_pidControllerRight.setP(SmartDashboard.getNumber("right p gain", 0));}
   if (righti != rightki) m_pidControllerRight.setI(SmartDashboard.getNumber("right i gain", 0));
  if (rightd != rightkd) { m_pidControllerRight.setD(SmartDashboard.getNumber("right d gain", 0));}
   if (rightff != rightkff){ m_pidControllerRight.setFF(SmartDashboard.getNumber("right feed foward", 0));}
   if (rightiz != rightkiz){ m_pidControllerRight.setIZone(SmartDashboard.getNumber("right i zone", 0));}
   if (rightmaxoutput != kMaxOutput) {rightmaxoutput = kMaxOutput;}
    




  }


  public double robotYaw(){
    return m_navX.getYaw();
  }

  public double robotPitch(){
    return m_navX.getPitch();
  }


}
