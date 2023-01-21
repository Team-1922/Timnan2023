// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  public static CANSparkMax m_leftLead = new CANSparkMax(Constants.kLeftLead, MotorType.kBrushless);
  public static CANSparkMax m_leftFollow = new CANSparkMax(Constants.kLeftFollow, MotorType.kBrushless);
  public static CANSparkMax m_rightLead = new CANSparkMax(Constants.kRightLead, MotorType.kBrushless);
  public static CANSparkMax m_rightFollow = new CANSparkMax(Constants.kRightFollow, MotorType.kBrushless);
  public SparkMaxPIDController m_pidControllerLeft;
  public SparkMaxPIDController m_pidControllerRight;
  double kp, ki, kd, kff, kiz, maxrpm;
 double rightkp, rightki, rightkd, rightkff, rightkiz, rightmaxrpm;
 double kMinOutput, kMaxOutput;
double RightkMinOutput, RightkMaxOutput;
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {


    SmartDashboard.putNumber("left p gain", kp);
    SmartDashboard.putNumber("left i gain", ki);
    SmartDashboard.putNumber("left d gain", kd); 
    SmartDashboard.putNumber("left feed foward ", kff);
    SmartDashboard.putNumber("left i zone", kiz);
    SmartDashboard.putNumber("left max rpm", maxrpm);
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
  m_pidControllerLeft.setP(SmartDashboard.getNumber( "left p gain" , 0));
  m_pidControllerLeft.setI(SmartDashboard.getNumber("left i gain", 0));
  m_pidControllerLeft.setD(SmartDashboard.getNumber("left d gain", 0));
 m_pidControllerLeft.setOutputRange(SmartDashboard.getNumber("left min output", 0), SmartDashboard.getNumber("left max output", 1)); 
 m_pidControllerLeft.setFF(SmartDashboard.getNumber("left feed foward", 0)); //feed foward
m_pidControllerLeft.setIZone(SmartDashboard.getNumber("left i zone", 0)); //i zone

double leftsetpoint = RobotContainer.LeftJoystick.getY()*maxrpm;
m_pidControllerLeft.setReference(leftsetpoint, CANSparkMax.ControlType.kVelocity);

     // right side
m_pidControllerRight.setP(SmartDashboard.getNumber("right p gain", 0));

m_pidControllerRight.setI(SmartDashboard.getNumber("right i gain", 0));
m_pidControllerRight.setD(SmartDashboard.getNumber("right d gain", 0));
m_pidControllerRight.setOutputRange(SmartDashboard.getNumber("right min output", 0), SmartDashboard.getNumber("right max output", 1));
m_pidControllerRight.setFF(SmartDashboard.getNumber("right feed foward", 0));
m_pidControllerRight.setIZone(SmartDashboard.getNumber("right i zone", 0));
double rightsetpoint = RobotContainer.RightJoystick.getY()*maxrpm;
m_pidControllerRight.setReference(rightsetpoint, CANSparkMax.ControlType.kVelocity);



 /* double MaxVelocity = 250; // the max velocity of the motor , test this when the drivebase is done
 double OutputScale = .9 ; //scale the output 
Drive( RobotContainer.LeftJoystick.getY()*MaxVelocity*OutputScale, RobotContainer.RightJoystick.getY()*MaxVelocity*OutputScale); */

}

public void DifferentialArcadeDrive(double rightside, double leftside){

DifferentialDrive.arcadeDriveIK(RobotContainer.LeftJoystick.getX(), RobotContainer.RightJoystick.getZ(), false);

}


public void DifferentialTankDrive(double rightside, double leftside){

DifferentialDrive.tankDriveIK(RobotContainer.LeftJoystick.getY(), RobotContainer.RightJoystick.getY(), false);
  
  }

  
public void DifferentialCurvatureDrive(double rightside, double leftside){

  DifferentialDrive.curvatureDriveIK(RobotContainer.LeftJoystick.getX(), RobotContainer.RightJoystick.getZ(), true);
  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  

  }


  public double robotYaw(){
    return m_navX.getYaw();
  }

  public double robotPitch(){
    return m_navX.getPitch();
  }


}
