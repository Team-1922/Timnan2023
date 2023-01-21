// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
  private CANSparkMax m_PivotArm = new CANSparkMax(Constants.kPivotMotorID, MotorType.kBrushless);
  //Put some encoder stuff in the future
  /** Creates a new ARM. */
  public Arm() {


    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
