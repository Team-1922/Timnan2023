// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class EndEffector extends SubsystemBase {
  private CANSparkMax m_BottomIOMotor = new CANSparkMax(Constants.kBottomIOMotorID, MotorType.kBrushless);
  private CANSparkMax m_TopIOMotor = new CANSparkMax(Constants.kTopIOMotorID, MotorType.kBrushless);
  SparkMaxPIDController m_BottomPID = m_BottomIOMotor.getPIDController();
  SparkMaxPIDController m_TopPID = m_TopIOMotor.getPIDController();
  public int m_valueRefCounter = 0;
  private double eP, eI, eD;

  private boolean m_hasObject;
  /** Creates a new EndEffector. */
  public EndEffector() {
    m_BottomIOMotor.restoreFactoryDefaults();
    m_TopIOMotor.restoreFactoryDefaults();
    m_BottomPID.setP(0);
    m_BottomPID.setI(0);
    m_BottomPID.setD(0);
    m_TopPID.setP(0);
    m_TopPID.setI(0);
    m_TopPID.setD(0); //Probably will be set on controller or determined through testing later
    
    SmartDashboard.putNumber("P gain", eP);
    SmartDashboard.putNumber("I gain", eI);
    SmartDashboard.putNumber("D gain", eD);
  }

  public boolean getHasObject() {
    return m_hasObject;
  }

  public void gatherTheCube() {
    Arm.m_ArmPID.setReference(Constants.kPivotMotorGatherAngle, ControlType.kPosition);
    m_TopPID.setReference(Constants.kIOMotorGatherPower, ControlType.kVoltage);
    m_BottomPID.setReference(Constants.kIOMotorGatherPower, ControlType.kVoltage);
    //Add some sensor stuff and conditionals
    m_hasObject = true;
  }

  public void Score(String scoreMode) {
    //Also needs some arm movement and calculations
    switch (scoreMode) {
      case "low":
      Arm.m_ArmPID.setReference(Constants.kPivotMotorLowAngle, ControlType.kPosition);
      m_TopPID.setReference(Constants.kIOMotorLowPower, ControlType.kVoltage);
      m_BottomPID.setReference(Constants.kIOMotorLowPower, ControlType.kVoltage);
      ;
      case "mid":
      Arm.m_ArmPID.setReference(Constants.kPivotMotorMidAngle, ControlType.kPosition);
      m_TopPID.setReference(Constants.kIOMotorMidPower, ControlType.kVoltage);
      m_BottomPID.setReference(Constants.kIOMotorMidPower, ControlType.kVoltage);
      ;
      case "high":
      Arm.m_ArmPID.setReference(Constants.kPivotMotorHighAngle, ControlType.kPosition);
      m_TopPID.setReference(Constants.kIOMotorHighPower, ControlType.kVoltage);
      m_BottomPID.setReference(Constants.kIOMotorHighPower, ControlType.kVoltage);
        //Avoiding the code for mid and high as of now since physical testing will be required to get accurate results
      ;
      default:
        System.out.println("Invalid Input")
      ;
      //more sensor stuff
      m_hasObject = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_valueRefCounter % Constants.eeRefRateMod == 0) {
     double ioP = SmartDashboard.getNumber("P gain", 0);
     if (eP != ioP) {m_BottomPID.setP(eP); m_TopPID.setP(eP); eP = ioP;}

    } else if (m_valueRefCounter % (Constants.eeRefRateMod + 1) == 0) {
      double ioI = SmartDashboard.getNumber("I gain", 0);
      if (eI != ioI) {m_BottomPID.setI(eI); m_TopPID.setI(eI); eI = ioI;}

    } else if (m_valueRefCounter % (Constants.eeRefRateMod + 2) == 0) {
      double ioD = SmartDashboard.getNumber("D gain", 0);
      if (eD != ioD) {m_BottomPID.setD(eD); m_TopPID.setD(eD); eD = ioD;}
    }
    m_valueRefCounter++;
  }
}
