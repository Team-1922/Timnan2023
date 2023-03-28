// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LightEmittingDiode;
import frc.robot.subsystems.ScoreMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSetMode extends InstantCommand {
  ScoreMode m_scoreMode;
  LightEmittingDiode m_LED;
  int m_mode;

  public AutoSetMode(ScoreMode scoreMode, LightEmittingDiode LED, int mode) {
    m_scoreMode = scoreMode;
    m_LED = LED;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_scoreMode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scoreMode.setScoreMode(m_mode);
    
      if(m_mode == 1){
        m_LED.setColor(0, 255, 0);
      } else if(m_mode == 2){
        m_LED.setColor(255, 200, 0);
      } else {
        m_LED.setColor(255, 0, 0);
      }
  }
}
