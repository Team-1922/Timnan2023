// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TrajectoryDrive;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrainSubsystem extends SubsystemBase { 
  public final static Field2d m_Field2d = new Field2d();
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
  private Pose2d SpotOne;
  double p=6e-5;
   double i=0;
   double d =0;
   double ff=0.000015;
   double iz = 0;
  public double Maxrpm = 2000;
  double rightp =  6e-5;
  double righti = 0;
  double rightd = 0;
  double rightff = 0.000015;
  double rightiz = 0;
 public double rightmaxrpm = 2000 ;
  double minoutput = -1;
  double maxoutput = 1;
  double rightminoutput = -1;
  double rightmaxoutput =1;
  Timer m_Timer;
  double JoystickDeadzone = 0.125;

  Timer balanceTimer = new Timer();
  /** Creates a new DriveTrainSubsystem. */
  


   // m_pigeon.calibrate();




  private double krightMinOutput; 
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(AHRS navX) {
    
    SmartDashboard.putNumber("joystick deadzone", JoystickDeadzone);
    
    SmartDashboard.putNumber("left p gain", p);
    SmartDashboard.putNumber("left i gain", i);
    SmartDashboard.putNumber("left d gain", d); 
    SmartDashboard.putNumber("left feed foward ", ff);
    SmartDashboard.putNumber("left i zone", iz);
    SmartDashboard.putNumber("left max rpm", Maxrpm);
    SmartDashboard.putNumber("left max output", maxoutput);
    SmartDashboard.putNumber("left min output", minoutput);

    
    SmartDashboard.putNumber("right p gain", rightp);
    SmartDashboard.putNumber("right i gain", righti);
    SmartDashboard.putNumber("right d gain", rightd); 
    SmartDashboard.putNumber("right feed foward ", rightff);
    SmartDashboard.putNumber("right i zone", rightiz);
    SmartDashboard.putNumber("right max rpm", rightmaxrpm);
    SmartDashboard.putNumber("right max output", rightmaxoutput);
    SmartDashboard.putNumber("right min output", rightminoutput);

    SmartDashboard.putData(m_Field2d);
    //Motor controlers
    m_leftLead.restoreFactoryDefaults();
    m_leftLead.setInverted(false);
    m_leftFollow.restoreFactoryDefaults();
    m_leftEncoder = m_leftLead.getEncoder();

    m_rightLead.restoreFactoryDefaults(); 
    m_rightLead.setInverted(true);
    m_rightFollow.restoreFactoryDefaults();
    m_rightEncoder = m_rightLead.getEncoder();
   

    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);

    m_pidControllerLeft = m_leftLead.getPIDController();
    m_pidControllerRight = m_rightLead.getPIDController();

    m_pidControllerLeft.setP(p);
    m_pidControllerLeft.setI(i);
    m_pidControllerLeft.setD(d);
   m_pidControllerLeft.setOutputRange(minoutput,maxoutput); 
   m_pidControllerLeft.setFF(ff); //feed foward
  m_pidControllerLeft.setIZone(iz); //i zone
  m_pidControllerRight.setP(rightp);
  m_pidControllerRight.setI(righti);
  m_pidControllerRight.setD(rightd);
  m_pidControllerRight.setOutputRange(rightminoutput, rightmaxoutput);
  m_pidControllerRight.setFF(rightff);
  m_pidControllerRight.setIZone(rightd);

  


  


    m_Timer = new Timer();
    m_Timer.start();
    m_navX = navX;

    // Setting up the odometry object 
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(robotYaw()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));
  }

public void oldVelocityDrive(double velocity){ //What's this
//  m_pidControllerLeft.setRefrence();
//  m_pidControllerRight.setRefrence();

 //m_pidControllerLeft.setRefrence();
  //m_pidControllerRight.setRefrence();


  }
  
public void Drive(double leftSpeed, double rightSpeed){
  m_leftLead.set(leftSpeed);
  m_rightLead.set(rightSpeed);

}
public void velocityDrive(double LeftRPM, double rightRPM){
m_pidControllerLeft.setReference(LeftRPM, CANSparkMax.ControlType.kVelocity);
m_pidControllerRight.setReference(rightRPM, CANSparkMax.ControlType.kVelocity);




 /* double MaxVelocity = 250; // the max velocity of the motor , test this when the drivebase is done
 double OutputScale = .9 ; //scale the output 
Drive( RobotContainer.LeftJoystick.getY()*MaxVelocity*OutputScale, RobotContainer.RightJoystick.getY()*MaxVelocity*OutputScale); */

}
public void timedpid( )    {
if (m_Timer.hasElapsed(5)){

  //left pid
  if (p != SmartDashboard.getNumber("left p gain", 6e-5)){ 
  p = SmartDashboard.getNumber("left p gain", 6e-5);
   m_pidControllerLeft.setP(p);
}
  if (i != SmartDashboard.getNumber("left i gain", 0)) { 
   i = SmartDashboard.getNumber("left i gain", 0); 
   m_pidControllerLeft.setI(i); }

  if (d != SmartDashboard.getNumber("left d gain", 0)){ 
   d = SmartDashboard.getNumber("left d gain", 0);
   m_pidControllerLeft.setD(d);}

  if (ff != SmartDashboard.getNumber("left feed foward", 0.000015)) {
    ff = SmartDashboard.getNumber("left feed foward", 0.000015);
    m_pidControllerLeft.setFF(ff); } 

   if (iz != SmartDashboard.getNumber("left i zone", 0)) {
    iz = SmartDashboard.getNumber("left i zone", 0);
    m_pidControllerLeft.setIZone(iz);}

  if (minoutput != SmartDashboard.getNumber("left min output", -1)){
    minoutput = SmartDashboard.getNumber("left min output", -1);
    m_pidControllerLeft.setOutputRange(minoutput, maxoutput);}

  if (maxoutput != SmartDashboard.getNumber("left max output", 1)){
    maxoutput = SmartDashboard.getNumber("left max output", 1);
    m_pidControllerLeft.setOutputRange(minoutput, maxoutput);}

  if (Maxrpm != SmartDashboard.getNumber("left max rpm", 5700)) {
    Maxrpm = SmartDashboard.getNumber("left max rpm", 5700);}


//right pid 
  if (rightp != SmartDashboard.getNumber("right p gain", 6e-5)){
    rightp = SmartDashboard.getNumber("right p gain", 6e-5);  
    m_pidControllerRight.setP(rightp);}
  
  if (righti != SmartDashboard.getNumber("right i gain", 0)) {
    righti = SmartDashboard.getNumber("right i gain", 0);
    m_pidControllerRight.setI(righti);}
  
  if (rightd != SmartDashboard.getNumber("right d gain", 0)) {
    rightd = SmartDashboard.getNumber("right d gain", 0);
    m_pidControllerRight.setD(rightd);}
  
  if (rightff != SmartDashboard.getNumber("right feed foward", 0.000015)){
    rightff = SmartDashboard.getNumber("right feed foward", 0.000015);
    m_pidControllerRight.setFF(rightff);}
  
  if (rightiz != SmartDashboard.getNumber("right i zone", 0)){
    rightiz = SmartDashboard.getNumber("right i zone", 0);
    m_pidControllerRight.setIZone(rightiz);}
  
  if (rightmaxoutput != SmartDashboard.getNumber("right max output", 1)) {
    rightmaxoutput = SmartDashboard.getNumber("right max output", 1);
    m_pidControllerRight.setOutputRange(rightminoutput, rightmaxoutput);}
  
  if (rightminoutput != SmartDashboard.getNumber("right min output", -1)) {
    krightMinOutput = SmartDashboard.getNumber("right min output", -1);
    m_pidControllerRight.setOutputRange(rightminoutput, rightmaxoutput);}

  if (rightmaxrpm != SmartDashboard.getNumber("right max rpm",5700 )) {
    rightmaxrpm = SmartDashboard.getNumber("right max rpm", 5700);}

m_Timer.reset();}
};

@Override
public void periodic()   {

    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(robotYaw()), Units.feetToMeters(getRightEncoderFeet()), Units.feetToMeters(getLeftEncoderFeet()));
    //this sets the starting position for the field image on the computer 
    // it still tracks the robot, but it seems to be off when it spins. 
    // also shuffleboard doesn't have this years game as an option to set the image as
    SpotOne = new Pose2d(-5, -5, Rotation2d.fromDegrees(0)).relativeTo(getRobotPose());
    // this puts it near the middle of the field
    m_Field2d.setRobotPose(getRobotPose().relativeTo(SpotOne));

    SmartDashboard.putNumber("LeftVelocity", m_leftEncoder.getVelocity());

    SmartDashboard.putNumber("PID Timer", m_Timer.get());
timedpid();

  
   //TEMP
   SmartDashboard.putNumber("RobotYaw", m_pigeon.getYaw() % 360);
   SmartDashboard.putNumber("RobotPitch", m_pigeon.getRoll());

   SmartDashboard.putNumber("EncoderLeft", m_leftEncoder.getPosition());

  }


  // Returns the navX Yaw, it's up and down like the way your neck moves 
  public double robotYaw(){
  //  return m_navX.getYaw();
      return m_pigeon.getYaw() % 360;
  }
  // Returns the navX Pitch, it's side to side like the way a turntable rotates
  public double robotPitch(){
  //  return m_navX.getPitch();
      return m_pigeon.getRoll();
  }


  public boolean balanceTimer(double seconds){
    SmartDashboard.putNumber("Balance timer", balanceTimer.get());
    return balanceTimer.get() >= seconds;
  }

  public void startBalance(){
    balanceTimer.start();
    balanceTimer.reset();

  }



  public double getLeftEncoderRaw(){
    return m_leftEncoder.getPosition();
  }
  public double getLeftEncoderFeet(){
    return m_leftEncoder.getPosition() * Constants.kEncoderRotationsToFeet;
  }

  public double getRightEncoderRaw(){
    return m_rightEncoder.getPosition();
  }
  public double getRightEncoderFeet(){
    return m_rightEncoder.getPosition() * Constants.kEncoderRotationsToFeet;
  }


  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }


}
