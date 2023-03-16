// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ScoreMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSetMode extends InstantCommand {
  ScoreMode m_scoreMode;
  int m_mode;
  public AutoSetMode(ScoreMode scoreMode, int mode) {
    m_scoreMode = scoreMode;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_scoreMode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scoreMode.setScoreMode(m_mode);
  }
}
