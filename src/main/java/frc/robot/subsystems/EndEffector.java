// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.playingwithfusion.TimeOfFlight;

public class EndEffector extends SubsystemBase {
  private static CANSparkMax m_BottomIOMotor = new CANSparkMax(Constants.kBottomIOMotorID, MotorType.kBrushless);
  private static CANSparkMax m_TopIOMotor = new CANSparkMax(Constants.kTopIOMotorID, MotorType.kBrushless);
  private static SparkMaxPIDController m_BottomPID = m_BottomIOMotor.getPIDController();
  private static SparkMaxPIDController m_TopPID = m_TopIOMotor.getPIDController();
  private static TimeOfFlight m_LeftSensor = new TimeOfFlight(Constants.kLeftSensorID);
  private static TimeOfFlight m_RightSensor = new TimeOfFlight(Constants.kRightSensorID);
  //Two motors needed on opposite sides, one higher up and one lower down.
  public static int m_valueRefCounter = 0;
  private double eP = .1, eI = 1e-4, eD = 1;
  public static int m_ScoreMode = -1;

  public static boolean m_hasObject;
  /** Creates a new EndEffector. */
  public EndEffector() {
    m_BottomIOMotor.restoreFactoryDefaults();
    m_TopIOMotor.restoreFactoryDefaults();
    m_BottomPID.setP(eP);
    m_BottomPID.setI(eI);
    m_BottomPID.setD(eD);
    m_TopPID.setP(eP);
    m_TopPID.setI(eI);
    m_TopPID.setD(eD); //Probably will be set on controller or determined through testing later
    
    SmartDashboard.putNumber("P gain", eP);
    SmartDashboard.putNumber("I gain", eI);
    SmartDashboard.putNumber("D gain", eD);

    m_LeftSensor.getRange();
  }

  public boolean getHasObject() {
    return m_hasObject;
  }

  public void stopMotors() {
    m_TopPID.setReference(0, ControlType.kVoltage);
    m_BottomPID.setReference(0, ControlType.kVoltage);
  }

  public void gatherTheCube() {
    m_TopPID.setReference(Constants.kIOMotorGatherPower*Constants.kIOBottomToTopVoltageConversion, ControlType.kVoltage);
    m_BottomPID.setReference(Constants.kIOMotorGatherPower, ControlType.kVoltage);
  }

  public void Score(String scoreMode) {
    switch (scoreMode) {
      case "low":
      m_TopPID.setReference(Constants.kIOMotorLowPower*Constants.kIOBottomToTopVoltageConversion, ControlType.kVoltage);
      m_BottomPID.setReference(Constants.kIOMotorLowPower, ControlType.kVoltage);
      ;
      case "mid":
      m_TopPID.setReference(Constants.kIOMotorMidPower*Constants.kIOBottomToTopVoltageConversion, ControlType.kVoltage);
      m_BottomPID.setReference(Constants.kIOMotorMidPower, ControlType.kVoltage);
      ;
      case "high":
      m_TopPID.setReference(Constants.kIOMotorHighPower*Constants.kIOBottomToTopVoltageConversion, ControlType.kVoltage);
      m_BottomPID.setReference(Constants.kIOMotorHighPower, ControlType.kVoltage);
        //Avoiding the code for mid and high as of now since physical testing will be required to get accurate results
      ;
      default:
        System.out.println("Invalid Input")
      ;
    }
  }

  //Add in some stuff to detect how far the cube is

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_valueRefCounter % Constants.eeRefRateMod == 0) {
     double ioP = SmartDashboard.getNumber("P gain", eP);
     if (eP != ioP) {m_BottomPID.setP(eP); m_TopPID.setP(eP); eP = ioP;}

    } else if (m_valueRefCounter % (Constants.eeRefRateMod) == 1) {
      double ioI = SmartDashboard.getNumber("I gain", eI);
      if (eI != ioI) {m_BottomPID.setI(eI); m_TopPID.setI(eI); eI = ioI;}

    } else if (m_valueRefCounter % (Constants.eeRefRateMod) == 2) {
      double ioD = SmartDashboard.getNumber("D gain", eD);
      if (eD != ioD) {m_BottomPID.setD(eD); m_TopPID.setD(eD); eD = ioD;}
    }
    m_valueRefCounter++;
  }
}
