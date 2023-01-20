// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class EndEffector extends SubsystemBase {
  private CANSparkMax m_RightIOMotor = new CANSparkMax(Constants.kRightIOMotorID, MotorType.kBrushless);
  private CANSparkMax m_LeftIOMotor = new CANSparkMax(Constants.kLeftIOMotorID, MotorType.kBrushless);
  private CANSparkMax m_RightHoldingMotor = new CANSparkMax(Constants.kRightHoldingMotorID, MotorType.kBrushless);
  private CANSparkMax m_LeftHoldingMotor = new CANSparkMax(Constants.kLeftHoldingMotorID, MotorType.kBrushless);
  private boolean m_hasObject;
  /** Creates a new EndEffector. */
  public EndEffector() {

  }

  public boolean getHasObject() {
    return m_hasObject;
  }

  public void gatherTheCube() {
    //We need the arm to move
    m_RightIOMotor.set(Constants.kIOMotorGatherRPM);
    m_LeftIOMotor.set(Constants.kIOMotorGatherRPM);
    //Add some sensor stuff and conditionals
    m_hasObject = true;
  }
  /*
  while (hasObject = true) {
    m_LeftHoldingMotor.set(Constants.kHoldingMotorRPM);
    mRightHoldingMotor.set(Constants.kHoldingMotorRPM);
  }
    Put this somewhere else later.
  */

  public void Score(String scoreMode) {
    //Also needs some arm movement and calculations
    switch (scoreMode) {
      case "low":
        
      ;
      case "mid":

      ;
      case "high":
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
