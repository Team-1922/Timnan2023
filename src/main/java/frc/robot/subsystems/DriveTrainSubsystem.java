// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrainSubsystem extends SubsystemBase { 
  public static CANSparkMax m_leftLead = new CANSparkMax(Constants.kLeftLead, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder;
  public static CANSparkMax m_leftFollow = new CANSparkMax(Constants.kLeftFollow, MotorType.kBrushless);
  public static CANSparkMax m_rightLead = new CANSparkMax(Constants.kRightLead, MotorType.kBrushless);
  private RelativeEncoder m_rightEncoder;
  public static CANSparkMax m_rightFollow = new CANSparkMax(Constants.kRightFollow, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerLeft;
  private SparkMaxPIDController m_pidControllerRight;


  private AHRS m_navX;
  private DifferentialDriveOdometry m_odometry;
  
  double kp, ki, kd, kff, kiz, kmaxrpm, rightkp, rightki, rightkd, rightkff, rightkiz, krightmaxrpm, kMinOutput, kMaxOutput,RightkMinOutput, RightkMaxOutput;
  double p,i,d,ff,iz,Maxrpm,rightp,righti,rightd,rightff,rightiz,rightmaxrpm, minoutput, maxoutput, rightminoutput, rightmaxoutput;
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
    SmartDashboard.putNumber("right max rpm", krightmaxrpm);
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


  private double krightMinOutput; 
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
    
    
    //pid stuff that we need
    //m_pidControllerLeft.setRefrence();
  //m_pidControllerRight.setRefrence();
  m_pidControllerLeft.setP(p);
  m_pidControllerLeft.setI(i);
  m_pidControllerLeft.setD(d);
 m_pidControllerLeft.setOutputRange(minoutput,maxoutput); 
 m_pidControllerLeft.setFF(ff); //feed foward
m_pidControllerLeft.setIZone(iz); //i zone
m_pidControllerRight.setP(rightp);
m_pidControllerRight.setI(righti);
m_pidControllerRight.setD(rightd);
m_pidControllerRight.setOutputRange(rightminoutput, RightkMaxOutput);
m_pidControllerRight.setFF(rightff);
m_pidControllerRight.setIZone(rightd);

    m_navX = navX;


    // Setting up the odometry object in the constructor--A little sketchy? No errors and it builds though
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(robotPitch()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));

  }
  
public void Drive(double leftSpeed, double rightSpeed){
  m_leftLead.set(leftSpeed);
  m_rightLead.set(rightSpeed);

}
public void velocityDrive(double LeftRPM, double rightRPM){



double leftsetpoint =LeftRPM;
m_pidControllerLeft.setReference(leftsetpoint, CANSparkMax.ControlType.kVelocity);
double rightsetpoint = rightRPM;
m_pidControllerRight.setReference(rightsetpoint, CANSparkMax.ControlType.kVelocity);



 /* double MaxVelocity = 250; // the max velocity of the motor , test this when the drivebase is done
 double OutputScale = .9 ; //scale the output 
Drive( RobotContainer.LeftJoystick.getY()*MaxVelocity*OutputScale, RobotContainer.RightJoystick.getY()*MaxVelocity*OutputScale); */

}

public void DifferentialArcadeDrive(double rightside, double leftside){

DifferentialDrive.arcadeDriveIK(RobotContainer.LeftJoystick.getX(), RobotContainer.LeftJoystick.getZ(), false);

}


public void DifferentialTankDrive(double rightside, double leftside){

DifferentialDrive.tankDriveIK(RobotContainer.LeftJoystick.getY(), RobotContainer.RightJoystick.getY(), false);
  
  }

  
public void DifferentialCurvatureDrive(double rightside, double leftside){

  DifferentialDrive.curvatureDriveIK(RobotContainer.LeftJoystick.getX(), RobotContainer.LeftJoystick.getZ(), true);
  
  }



public void timedpid( )   {


Timer.delay(2);

    kp = SmartDashboard.getNumber("left p gain", 0);
    ki= SmartDashboard.getNumber("left i gain", 0);
    kd = SmartDashboard.getNumber("left d gain", 0);
    kmaxrpm = SmartDashboard.getNumber("left max rpm", 10);

Timer.delay(2);
 kff = SmartDashboard.getNumber("left feed foward", 0);
 kiz = SmartDashboard.getNumber("left i zone", 0);
kMinOutput = SmartDashboard.getNumber("left min output", 0);
kMaxOutput = SmartDashboard.getNumber("left max output", 1);
Timer.delay(2);

rightkp = SmartDashboard.getNumber("right p gain", 0);
rightki = SmartDashboard.getNumber("right i gain", 0);
rightkd = SmartDashboard.getNumber("right d gain", 0);
Timer.delay(2);
  rightkff = SmartDashboard.getNumber("right feed foward", 0);
  rightkiz = SmartDashboard.getNumber("right iz", 0);
  RightkMaxOutput = SmartDashboard.getNumber("right max output", 1);
  RightkMinOutput = SmartDashboard.getNumber("right min output", 0);
  krightmaxrpm = SmartDashboard.getNumber("right max rpm", 10); 


    // This method will be called once per scheduler run

timedpid();
    //left pid

  if (p != kp){  m_pidControllerLeft.setP(SmartDashboard.getNumber( "left p gain" , 0)); }
if (i != ki) {  m_pidControllerLeft.setI(SmartDashboard.getNumber("left i gain", 0)); }
 if (d != kd){ m_pidControllerLeft.setD(SmartDashboard.getNumber("left d gain", 0));}
  if (ff != kff) {m_pidControllerLeft.setFF(SmartDashboard.getNumber("left feed foward", 0)); } //feed foward
   if (iz != kiz) {m_pidControllerLeft.setIZone(SmartDashboard.getNumber("left i zone", 0));}
  if (minoutput != kMinOutput){minoutput = SmartDashboard.getNumber("left min output", 0);}
  if (maxoutput != kMaxOutput){maxoutput = SmartDashboard.getNumber("left max output", 1);}
if (Maxrpm != kmaxrpm) {Maxrpm = SmartDashboard.getNumber("left max rpm", 10);}

   // right pid
   if (rightp != rightkp){ m_pidControllerRight.setP(SmartDashboard.getNumber("right p gain", 0));}
   if (righti != rightki) m_pidControllerRight.setI(SmartDashboard.getNumber("right i gain", 0));
   if (rightd != rightkd) { m_pidControllerRight.setD(SmartDashboard.getNumber("right d gain", 0));}
   if (rightff != rightkff){ m_pidControllerRight.setFF(SmartDashboard.getNumber("right feed foward", 0));}
   if (rightiz != rightkiz){ m_pidControllerRight.setIZone(SmartDashboard.getNumber("right i zone", 0));}
   if (RightkMaxOutput != rightmaxoutput) {rightmaxoutput =SmartDashboard.getNumber("right max output", 1);}
   if (rightminoutput != krightMinOutput) {krightMinOutput = SmartDashboard.getNumber("right min output", 0);}
   if (rightmaxrpm != krightmaxrpm) {rightmaxrpm = SmartDashboard.getNumber("right max rpm", 10);}



    m_odometry.update(Rotation2d.fromDegrees(robotPitch()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));
  


    //left pid
  if (p != kp){  m_pidControllerLeft.setP(SmartDashboard.getNumber( "left p gain" , 0)); }
if (i != ki) {  m_pidControllerLeft.setI(SmartDashboard.getNumber("left i gain", 0)); }
 if (d != kd){ m_pidControllerLeft.setD(SmartDashboard.getNumber("left d gain", 0));}
  if (ff != kff) {m_pidControllerLeft.setFF(SmartDashboard.getNumber("left feed foward", 0)); } //feed foward
   if (iz != kiz) {m_pidControllerLeft.setIZone(SmartDashboard.getNumber("left i zone", 0));}
  if (minoutput != kMinOutput){minoutput = SmartDashboard.getNumber("left min output", 0);}
  if (maxoutput != kMaxOutput){maxoutput = SmartDashboard.getNumber("left max output", 1);}
if (Maxrpm != kmaxrpm) {Maxrpm = SmartDashboard.getNumber("left max rpm", 10);}

   // right pid
   if (rightp != rightkp){ m_pidControllerRight.setP(SmartDashboard.getNumber("right p gain", 0));}
   if (righti != rightki) m_pidControllerRight.setI(SmartDashboard.getNumber("right i gain", 0));
   if (rightd != rightkd) { m_pidControllerRight.setD(SmartDashboard.getNumber("right d gain", 0));}
   if (rightff != rightkff){ m_pidControllerRight.setFF(SmartDashboard.getNumber("right feed foward", 0));}
   if (rightiz != rightkiz){ m_pidControllerRight.setIZone(SmartDashboard.getNumber("right i zone", 0));}
   if (RightkMaxOutput != rightmaxoutput) {rightmaxoutput =SmartDashboard.getNumber("right max output", 1);}
   if (krightMinOutput != rightminoutput) {krightMinOutput = SmartDashboard.getNumber("right min output", 0);}
   if (rightmaxrpm != krightmaxrpm) {rightmaxrpm = SmartDashboard.getNumber("right max rpm", 10);}



  }



  private void WaitCommand(int j)
   {
    new WaitCommand(j);
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
    return m_leftEncoder.getPosition() * Constants.kEncoderTicksToFeet;
  }

  public double getRightEncoderRaw(){
    return m_rightEncoder.getPosition();
  }
  public double getRightEncoderFeet(){
    return m_rightEncoder.getPosition() * Constants.kEncoderTicksToFeet;
  }


  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }


}
