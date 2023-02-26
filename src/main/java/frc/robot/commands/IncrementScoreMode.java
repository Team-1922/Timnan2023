// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmitingDiode;
import frc.robot.subsystems.ScoreMode;

public class IncrementScoreMode extends CommandBase {
  /** Creates a new increaseScoreMode. */
  private ScoreMode m_Score;
  public IncrementScoreMode(ScoreMode Score) {
  
    // Use addRequirements() here to declare subsystem dependencies.
    m_Score = Score;
    
    addRequirements(Score);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Score.incrementScoreMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_Score.getScoreMode()== 1){}
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
