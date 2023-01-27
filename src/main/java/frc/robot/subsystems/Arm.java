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
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Arm extends SubsystemBase {
  static CANSparkMax m_PivotArm = new CANSparkMax(Constants.kPivotMotorID, MotorType.kBrushless);
  static SparkMaxAbsoluteEncoder m_ArmEncoder = m_PivotArm.getAbsoluteEncoder(null);
  static SparkMaxPIDController m_ArmPID = m_PivotArm.getPIDController();
  static double m_ArmPos = m_ArmEncoder.getPosition();
  public static int m_valueRefCounter = EndEffector.m_valueRefCounter;
  public static double aP, aI, aD, aFF;
  //Put some encoder stuff in the future
  /** Creates a new ARM. */
  public Arm() {
    m_ArmPID.setOutputRange(Constants.kPivotMotorMinAngle, Constants.kPivotMotorMaxAngle);
    m_ArmPID.setP(0);
    m_ArmPID.setI(0);
    m_ArmPID.setD(0);
    m_ArmPID.setFF(0); //Probably will be set on controller or determined through testing later
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_valueRefCounter % Constants.eeRefRateMod == 0) {
      double aeP = SmartDashboard.getNumber("P gain", 0);
      if (aP != aeP) {m_ArmPID.setP(aeP); aP = aeP;}
      double aeFF = SmartDashboard.getNumber("FF gain", 0);
      if (aFF != aeFF) {m_ArmPID.setFF(aeFF); aFF = aeFF;}
 
     } else if (m_valueRefCounter % (Constants.eeRefRateMod + 1) == 0) {
       double aeI = SmartDashboard.getNumber("I gain", 0);
       if (aI != aeI) {m_ArmPID.setI(aeI); aI = aeI;}
 
     } else if (m_valueRefCounter % (Constants.eeRefRateMod + 2) == 0) {
       double aeD = SmartDashboard.getNumber("D gain", 0);
       if (aD != aeD) {m_ArmPID.setD(aeD); aD = aeD;}

     }
     m_valueRefCounter++;
  }

  public double setNewFF() {
    double newFF;
    newFF = (Constants.kCOMRadius * Math.cos(m_ArmEncoder.getPosition()*2*Math.PI));  //one rotation is 2pi 
    return newFF;
  }
}
