// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class EndEffector extends SubsystemBase {
  private CANSparkMax m_RightIOMotor = new CANSparkMax(Constants.kRightIOMotorID);
  private CANSparkMax m_LeftIOMotor = new CANSparkMax(Constants.kLeftIOMotorID);
  private CANSparkMax m_RightHoldingMotor = new CANSparkMax(Constants.kRightHoldingMotorID);
  private CANSparkMax m_LeftHoldingMotor = new CANSparkMax(Constants.kRightIOLeftHoldingMotorID);
  /** Creates a new EndEffector. */
  public EndEffector() {}

  public void GatherTheCube() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
