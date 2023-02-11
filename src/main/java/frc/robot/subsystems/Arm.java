// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Arm extends SubsystemBase {
  private static CANSparkMax m_Arm = new CANSparkMax(Constants.kPivotMotorID, MotorType.kBrushless);
  private static SparkMaxAbsoluteEncoder m_ArmEncoder = m_Arm.getAbsoluteEncoder(Type.kDutyCycle);
  private static SparkMaxPIDController m_ArmPID = m_Arm.getPIDController();
  private int m_valueRefCounter;
  public double aP = .1, aI = 1e-4, aD = 1, aFF = 1;
  //Put some encoder stuff in the future
  /** Creates a new ARM. */
  public Arm() {
    m_Arm.restoreFactoryDefaults();
    m_ArmPID.setOutputRange(Constants.kPivotMotorMinAngle, Constants.kPivotMotorMaxAngle);
    m_ArmPID.setP(aP);
    m_ArmPID.setI(aI);
    m_ArmPID.setD(aD);
    m_ArmPID.setFF(aFF); //Probably will be set on controller or determined through testing later
    m_ArmPID.getIZone();
    m_ArmEncoder.setZeroOffset(0);
    m_ArmEncoder.setInverted(false);
    m_ArmPID.setFeedbackDevice(m_ArmEncoder);
    m_ArmEncoder.setPositionConversionFactor(Constants.kPositionConversionFactor);
    m_ArmEncoder.setVelocityConversionFactor(Constants.kVelocityConversionFactor);
    //PID wrapping stuff
    m_ArmPID.setPositionPIDWrappingEnabled(true);
    m_ArmPID.setPositionPIDWrappingMinInput(Constants.kWrappedPIDMinInput);
    m_ArmPID.setPositionPIDWrappingMaxInput(Constants.kWrappedPIDMaxInput);

    m_valueRefCounter = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  }

  public double setNewFF() {
    double newFF;
    newFF = (Constants.kCOMRadius * Math.cos(m_ArmEncoder.getPosition()*2*Math.PI));  //one rotation is 2pi 
    return newFF;
  }

  public double getAngle(boolean inDeg) {
    if (inDeg) {
      return (m_ArmEncoder.getPosition()*360);
    } else return (m_ArmEncoder.getPosition()*2*Math.PI);
    
  }

  public double setAngle(double finalAngle /* Use degrees */)  {
    double currentAngle = m_ArmEncoder.getPosition();
    if (currentAngle != (finalAngle / 360)) m_ArmPID.setReference(finalAngle, ControlType.kPosition);
    return finalAngle;
  }
}
