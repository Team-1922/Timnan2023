// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreMode extends SubsystemBase {
    private int m_ScoreMode;
    public ScoreMode() {
        m_ScoreMode = 1;
    }

    public int getScoreMode() {
        return m_ScoreMode;
    }

    public void setScoreMode(int mode){
        m_ScoreMode = mode;
    }

    public void incrementScoreMode() {
        if (m_ScoreMode >= 3) {
            m_ScoreMode = 1;
        } else m_ScoreMode++;
        
    }

    public void incrementScoreModeDown(){
        if(m_ScoreMode <= 1){
            m_ScoreMode = 3;
        } else m_ScoreMode--;
    }



    
}