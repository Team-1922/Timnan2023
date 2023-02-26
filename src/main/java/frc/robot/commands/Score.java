// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ScoreMode;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LightEmitingDiode;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Score extends CommandBase {
  private EndEffector m_EndEffector;
  private Arm m_Arm;
  private ScoreMode m_ScoreMode;
  private int scoreMode;
  private double finalAngle;
  private  LightEmitingDiode m_LightEmitingDiode;
  /** Creates a new Score. */
  public Score(Arm pivotArm, EndEffector cubeEffector, ScoreMode score, LightEmitingDiode LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = pivotArm;
    m_EndEffector = cubeEffector;
    m_ScoreMode = score;
    m_LightEmitingDiode = LED;
    addRequirements(pivotArm);
    addRequirements(cubeEffector);
    addRequirements(score);
    addRequirements(LED);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoreMode = m_ScoreMode.getScoreMode();
    if (scoreMode == 1) {m_Arm.setAngle(Constants.kPivotMotorLowAngle); m_LightEmitingDiode.setColor(255, 0,0);
    } else if (scoreMode == 2) {m_Arm.setAngle(Constants.kPivotMotorMidAngle); m_LightEmitingDiode.setColor(0,255,0);
    } else if (scoreMode == 3) {m_Arm.setAngle(Constants.kPivotMotorHighAngle);  m_LightEmitingDiode.setColor(0, 0,255);}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (scoreMode == 1) {m_EndEffector.Score("low"); m_LightEmitingDiode.setColor(255,0,0);
    } else if (scoreMode == 2) {m_EndEffector.Score("mid"); m_LightEmitingDiode.setColor(0,255,0);
    } else if (scoreMode == 3) {m_EndEffector.Score("high"); m_LightEmitingDiode.setColor(0,0,255);}
    Timer.delay(1);
    //May use start command
    m_EndEffector.stopMotors();
    m_Arm.setAngle(Constants.kPivotMotorLowAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Timer.delay(.2);
    finalAngle = Arm.m_FinalAngle;
    return (Math.abs(finalAngle - m_Arm.getPosition()) <= 3);
  }
}
