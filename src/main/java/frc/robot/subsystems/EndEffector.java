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
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;

public class EndEffector extends SubsystemBase {
  private CANSparkMax m_RightIOMotor = new CANSparkMax(Constants.kRightIOMotorID, MotorType.kBrushless);
  private SparkMaxPIDController m_RightPID = m_RightIOMotor.getPIDController();
  private CANSparkMax m_LeftIOMotor = new CANSparkMax(Constants.kLeftIOMotorID, MotorType.kBrushless);
  private SparkMaxPIDController m_LeftPID = m_LeftIOMotor.getPIDController();
  private CANSparkMax pivotMotor = Arm.m_PivotArm;

  private boolean m_hasObject;
  /** Creates a new EndEffector. */
  public EndEffector() {

  }

  public boolean getHasObject() {
    return m_hasObject;
  }

  public void gatherTheCube() {
    Arm.m_ArmPivoter.setReference(Constants.kPivotMotorGatherAngle, ControlType.kPosition);
    m_LeftPID.setReference(Constants.kIOMotorGatherPower, ControlType.kVoltage);
    m_RightPID.setReference(Constants.kIOMotorGatherPower, ControlType.kVoltage);
    //Add some sensor stuff and conditionals
    m_hasObject = true;
  }

  public void Score(String scoreMode) {
    //Also needs some arm movement and calculations
    switch (scoreMode) {
      case "low":
      Arm.m_ArmPivoter.setReference(Constants.kPivotMotorLowAngle, ControlType.kPosition);
      m_LeftPID.setReference(Constants.kIOMotorLowPower, ControlType.kVoltage);
      m_RightPID.setReference(Constants.kIOMotorLowPower, ControlType.kVoltage);
      ;
      case "mid":
      Arm.m_ArmPivoter.setReference(Constants.kPivotMotorMidAngle, ControlType.kPosition);
      m_LeftPID.setReference(Constants.kIOMotorMidPower, ControlType.kVoltage);
      m_RightPID.setReference(Constants.kIOMotorMidPower, ControlType.kVoltage);
      ;
      case "high":
      Arm.m_ArmPivoter.setReference(Constants.kPivotMotorHighAngle, ControlType.kPosition);
      m_LeftPID.setReference(Constants.kIOMotorHighPower, ControlType.kVoltage);
      m_RightPID.setReference(Constants.kIOMotorHighPower, ControlType.kVoltage);
        //Avoiding the code for mid and high as of now since physical testing will be required to get accurate results
      ;
      default:
        System.out.println("Invalid Input")
      ;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
