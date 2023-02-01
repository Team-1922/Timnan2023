// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenixpro.hardware.Pigeon2;
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

  private Pigeon2 m_pigeon = new Pigeon2(Constants.kPigeon);

  private AHRS m_navX;
  private DifferentialDriveOdometry m_odometry;
  
  double kp, ki, kd, kff, kiz, kmaxrpm, rightkp, rightki, rightkd, rightkff, rightkiz, krightmaxrpm, kMinOutput, kMaxOutput,RightkMinOutput, RightkMaxOutput;
  double p,i,d,ff,iz,Maxrpm,rightp,righti,rightd,rightff,rightiz,rightmaxrpm, minoutput, maxoutput, rightminoutput, rightmaxoutput;
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {


    m_pigeon.calibrate();

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

    // Setting up the odometry object 
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(robotPitch()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));
  }
  
public void Drive(double leftSpeed, double rightSpeed){
  m_leftLead.set(leftSpeed);
  m_rightLead.set(rightSpeed);

}
public void velocityDrive(double velocity){
//  m_pidControllerLeft.setRefrence();
//  m_pidControllerRight.setRefrence();

 //m_pidControllerLeft.setRefrence();
  //m_pidControllerRight.setRefrence();
  m_pidControllerLeft.setP(p);
  m_pidControllerLeft.setI(i);
  m_pidControllerLeft.setD(d);
 m_pidControllerLeft.setOutputRange(minoutput,maxoutput); 
 m_pidControllerLeft.setFF(ff); //feed foward
m_pidControllerLeft.setIZone(iz); //i zone

double leftsetpoint = RobotContainer.LeftJoystick.getY()*Maxrpm;
m_pidControllerLeft.setReference(leftsetpoint, CANSparkMax.ControlType.kVelocity);

     // right side
m_pidControllerRight.setP(rightp);

m_pidControllerRight.setI(righti);
m_pidControllerRight.setD(rightd);
m_pidControllerRight.setOutputRange(rightminoutput, RightkMaxOutput);
m_pidControllerRight.setFF(rightff);
m_pidControllerRight.setIZone(rightd);
double rightsetpoint = RobotContainer.RightJoystick.getY()*rightmaxrpm;
m_pidControllerRight.setReference(rightsetpoint, CANSparkMax.ControlType.kVelocity);




 /* double MaxVelocity = 250; // the max velocity of the motor , test this when the drivebase is done
 double OutputScale = .9 ; //scale the output 
Drive( RobotContainer.LeftJoystick.getY()*MaxVelocity*OutputScale, RobotContainer.RightJoystick.getY()*MaxVelocity*OutputScale); */

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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


  // Returns the navX Yaw, it's up and down like the way your neck moves 
  public double robotYaw(){
  //  return m_navX.getYaw();
      return m_pigeon.getYaw().getValue();
  }
  // Returns the navX Pitch, it's side to side like the way a turntable rotates
  public double robotPitch(){
  //  return m_navX.getPitch();
      return m_pigeon.getPitch().getValue();
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
