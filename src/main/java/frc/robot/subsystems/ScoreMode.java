// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreMode extends SubsystemBase {
    public static int m_ScoreMode = 1;
    public ScoreMode() {}

    public int getScoreMode() {
        return m_ScoreMode;
    }
}