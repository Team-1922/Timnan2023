// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ScoreMode;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Score extends CommandBase {
  private EndEffector m_EndEffector;
  private Arm m_Arm;
  private ScoreMode m_ScoreMode;
  private int scoreMode;
  
  /** Creates a new Score. */
  public Score(Arm pivotArm, EndEffector cubeEffector, ScoreMode score) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = pivotArm;
    m_EndEffector = cubeEffector;
    m_ScoreMode = score;
    
    addRequirements(pivotArm);
    addRequirements(cubeEffector);
    addRequirements(score);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {scoreMode = m_ScoreMode.getScoreMode();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setNewFF();
    if (scoreMode == 1) {m_Arm.setAngle(Constants.kPivotMotorLowAngle); m_EndEffector.Score("low");
    } else if (scoreMode == 2) {m_Arm.setAngle(Constants.kPivotMotorMidAngle); m_EndEffector.Score("mid");
    } else if (scoreMode == 3) {m_Arm.setAngle(Constants.kPivotMotorHighAngle); m_EndEffector.Score("high");}
      //sensor stuff
      EndEffector.m_hasObject = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffector.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_EndEffector.stopMotors();
    m_Arm.setAngle(Constants.kPivotMotorStowAngle);
    return m_EndEffector.getHasObject();
  }
}
