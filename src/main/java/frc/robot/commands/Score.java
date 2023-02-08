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
  private EndEffector m_CubePositioner;
  private Arm m_RobotArm;
  private ScoreMode m_ScoreMode;
  private int scoreMode;
  
  /** Creates a new Score. */
  public Score(Arm pivotArm, EndEffector cubeEffector, ScoreMode score) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RobotArm = pivotArm;
    m_CubePositioner = cubeEffector;
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
    m_RobotArm.setNewFF();
    if (scoreMode == 1) {m_RobotArm.setAngle(Constants.kPivotMotorLowAngle); m_CubePositioner.Score("low");
    } else if (scoreMode == 2) {m_RobotArm.setAngle(Constants.kPivotMotorMidAngle); m_CubePositioner.Score("mid");
    } else if (scoreMode == 3) {m_RobotArm.setAngle(Constants.kPivotMotorHighAngle); m_CubePositioner.Score("high");}
    while (EndEffector.m_hasObject) {
      //sensor stuff
      EndEffector.m_hasObject = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_RobotArm.setAngle(Constants.kPivotMotorStowAngle);
    return (!m_CubePositioner.getHasObject());
  }
}
