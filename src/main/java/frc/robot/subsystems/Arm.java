// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Arm extends SubsystemBase {
  private static CANCoder m_CANCoder = new CANCoder(0);
  private static CANCoderConfiguration m_config = new CANCoderConfiguration();
  
  private static CANSparkMax m_Arm = new CANSparkMax(Constants.kPivotMotorID, MotorType.kBrushless);
  
  private static SparkMaxAbsoluteEncoder m_ArmEncoder = m_Arm.getAbsoluteEncoder(Type.kDutyCycle);
  private static SparkMaxPIDController m_ArmPID = m_Arm.getPIDController();
  public double m_Position;
  public double[] m_StartingVector = new double[2];
  public double[] m_TargetVector = new double[2];
  public double [] m_MidVector = new double[2];
  public double m_CalculatedVoltage;
  private int m_valueRefCounter;
  public double aP = .0046, aI = 0, aD = 0.05, aFF = 0; //p .0055 d .08
  
 // private TimeOfFlight m_TOF = new TimeOfFlight(Constants.kFrontSensorID);
  
  // public double aP = .0025, aI = 16e-7, aD = 0.016, aFF = 2e-6;
  public static double m_FinalAngle;
  //Put some encoder stuff in the future
  /** Creates a new ARM. */
  public Arm() {
    m_config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    m_config.magnetOffsetDegrees = Constants.kZeroOffset;
    m_config.sensorCoefficient = 1000 / 4096.0;
    m_config.sensorTimeBase = SensorTimeBase.PerSecond; //The time frame in which velocity will be measured (if we ever need it)
    m_CANCoder.configAllSettings(m_config);

    m_Arm.restoreFactoryDefaults();
    m_Arm.setInverted(true);
    m_ArmPID.setP(aP);
    SmartDashboard.putNumber("P gain", aP);
    m_ArmPID.setI(aI);
    SmartDashboard.putNumber("I gain", aI);
    m_ArmPID.setD(aD);
    SmartDashboard.putNumber("D gain", aD);
    m_ArmPID.setFF(aFF);
    SmartDashboard.putNumber("FF gain", aFF);
    m_ArmPID.getIZone();
    m_ArmPID.setFeedbackDevice(m_ArmEncoder);
    m_ArmEncoder.setInverted(false);
    m_ArmEncoder.setPositionConversionFactor(Constants.kPositionConversionFactor);
    m_ArmEncoder.setZeroOffset(Constants.kZeroOffset);
    m_valueRefCounter = 0;
  SmartDashboard.putNumber("angle", m_CANCoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    if (m_valueRefCounter % Constants.eeRefRateMod == 0) {
      double kP = SmartDashboard.getNumber("P gain", aP);
      if (aP != kP) {m_ArmPID.setP(kP); aP = kP;}
      double kFF = SmartDashboard.getNumber("FF gain", aFF);
      if (aFF != kFF) {m_ArmPID.setFF(kFF); aFF = kFF;}
 
     } else if (m_valueRefCounter % Constants.eeRefRateMod == 2) {
       double kI = SmartDashboard.getNumber("I gain", aI);
       if (aI != kI) {m_ArmPID.setI(kI); aI = kI;}
 
     } else if (m_valueRefCounter % Constants.eeRefRateMod == 3) {
       double kD = SmartDashboard.getNumber("D gain", aD);
       if (aD != kD) {m_ArmPID.setD(kD); aD = kD;}

     }
     m_valueRefCounter++;
     */
   //  SmartDashboard.putBoolean("Has Cube?", m_TOF.getRange() <= 50);
    // SmartDashboard.putNumber("TOF Value", m_TOF.getRange());

  }

  public double getPosition() {
    return m_CANCoder.getPosition();
  }

  public double setAngle(double finalAngle)  {
    m_FinalAngle = finalAngle;
    m_ArmPID.setReference(finalAngle, ControlType.kPosition);
    SmartDashboard.putNumber("Target angle", finalAngle);
    return m_FinalAngle;
  }

  public double[][] calculateVoltage(double finalAngle, double voltageDialation, double[][] combinedVectors) {
    m_Position = m_CANCoder.getPosition();
    m_StartingVector[0] = m_Position;
    m_StartingVector[1] = 0.5;
    m_MidVector[0] = (finalAngle-m_Position)/2;
    m_MidVector[1] = (12*voltageDialation);
    m_TargetVector[0] = finalAngle;
    m_TargetVector[1] = 0.5;
    combinedVectors[0][0] = m_StartingVector[0];
    combinedVectors[0][1] = m_StartingVector[1];
    combinedVectors[1][0] = m_MidVector[0];
    combinedVectors[1][1] = m_MidVector[1];
    combinedVectors[2][0] = m_TargetVector[0];
    combinedVectors[2][1] = m_TargetVector[1];
    //May consider removing the matrix in the future if it saves resources.
    
    return combinedVectors;
  }

  public void setVoltage(double Voltage) {
    m_ArmPID.setReference(Voltage, ControlType.kVoltage);
  }


/* 
  public double getTOF(){
    return m_TOF.getRange();
  }
*/
/*   public boolean hasCube(){
    // Threshold for no cube is about 365
    // Threshold for yes, cube, farthest away possible is 145
    return (m_TOF.getRange() <= 145);// 145
  } */
}
