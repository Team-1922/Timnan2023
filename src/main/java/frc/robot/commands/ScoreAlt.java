// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.ScoreMode;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreAlt extends CommandBase {
  private EndEffector m_EndEffector;
  private Arm m_Arm;
  private ScoreMode m_ScoreMode;
  
  public static double[][] m_BaselineVectors = new double[3][2];
  public double m_CalculatedVoltage;
  public double m_Difference;
  public double m_SMode;
  public String m_ShootingSpeed;
  public double m_Position;
  
  /** Creates a new Score. */
  public ScoreAlt(Arm pivotArm, EndEffector cubeEffector, ScoreMode score) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = pivotArm;
    m_EndEffector = cubeEffector;
    m_ScoreMode = score;
    //m_LightEmitingDiode = LED;
    addRequirements(pivotArm);
    addRequirements(cubeEffector);
    addRequirements(score);
    //addRequirements(LED);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int m_SMode = m_ScoreMode.getScoreMode();
    if (m_SMode == 1) {m_Arm.calculateVoltage(Constants.kPivotMotorLowAngle, .25, m_BaselineVectors);
      m_ShootingSpeed = "low";
    }
    else if (m_SMode == 2) {m_Arm.calculateVoltage(Constants.kPivotMotorMidAngle, .7, m_BaselineVectors);
      m_ShootingSpeed = "mid";
    }
    else if ( m_SMode == 3) {m_Arm.calculateVoltage(Constants.kPivotMotorHighAngle, .3, m_BaselineVectors);
      m_ShootingSpeed = "high";
    }
    System.out.println(m_BaselineVectors[2][0]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Position = m_Arm.getPosition();
    if (m_Arm.getPosition() <= m_BaselineVectors[1][0]) {
        m_CalculatedVoltage = (.25+m_BaselineVectors[1][1] - m_BaselineVectors[1][1]*(m_BaselineVectors[1][0]-m_Position)/(m_BaselineVectors[1][0]*.9));
    } else {
        m_CalculatedVoltage = (.25+m_BaselineVectors[1][1] - m_BaselineVectors[1][1]*(m_Position-m_BaselineVectors[1][0])/(m_Position*.9));
    }
    m_Arm.setVoltage(m_CalculatedVoltage);
    System.out.println(m_Position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) m_EndEffector.Score(m_ShootingSpeed);
    Timer.delay(.4);
    m_EndEffector.stopMotors();
    m_Arm.setAngle(Constants.kPivotMotorLowAngle);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 2; j++) {
        m_BaselineVectors[i][j] = 0;
      }
    }
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_Position = m_Arm.getPosition();
    return (Math.abs(m_BaselineVectors[2][0]-m_Position) <= 6);
  }
}