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
  public double aP = .0025, aI = 0, aD = 0.00, aFF = 0;
  // public double aP = .0025, aI = 16e-7, aD = 0.016, aFF = 2e-6;
  public static double m_FinalAngle;
  //Put some encoder stuff in the future
  /** Creates a new ARM. */
  public Arm() {
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
     SmartDashboard.putNumber("Arm angle",m_ArmEncoder.getPosition());
  }

  public double getPosition() {
    return m_ArmEncoder.getPosition();
  }

  public double setAngle(double finalAngle)  {
    m_FinalAngle = finalAngle;
    m_ArmPID.setReference(finalAngle, ControlType.kPosition);
    System.out.println("Angle is now being set.");
    System.out.println(m_ArmEncoder.getPosition());
    System.out.println(finalAngle);
    SmartDashboard.putNumber("Target angle", finalAngle);
    return m_FinalAngle;
  }
}
