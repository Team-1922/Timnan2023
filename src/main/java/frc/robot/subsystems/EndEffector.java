// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private boolean m_hasObject;
  /** Creates a new EndEffector. */
  public EndEffector() {
    m_BottomIOMotor.restoreFactoryDefaults();
    m_TopIOMotor.restoreFactoryDefaults();
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
  }
}
