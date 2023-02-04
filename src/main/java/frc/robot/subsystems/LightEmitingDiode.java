// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LightEmitingDiode extends SubsystemBase {
  private int SCORE_LOW = 0;
  private int SCORE_MED = 1;
  private int SCORE_HIGH = 2;

  private int SCORE_INACTIVE = 0;
  private int SCORE_ACTIVE = 1;
  private int SCORE_DONE = 2;
  
  private int m_scoreState;
  private int m_scoreMode;
  
  private int COLLECT_INACTIVE = 0;
  private int COLLECT_ACTIVE = 1;
  private int COLLECT_DONE = 2;

  private int m_collectMode;

  private final CANdle m_candle = new CANdle(Constants.kCandleId);

  private final int m_yellowR = 255;
  private final int m_yellowG = 255;
  private final int m_yellowB = 0;
  private final int m_greenR = 0;
  private final int m_greenG = 255;
  private final int m_greenB = 0;


  /** Creates a new LED. */
  public LightEmitingDiode() {
    m_scoreState = SCORE_LOW;
    m_scoreMode = SCORE_INACTIVE;
    m_collectMode = COLLECT_INACTIVE;
// we can either use the leds as a status indicator, or as just a decoration, both if we can figure that out in our spare line.


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_collectMode != COLLECT_INACTIVE) {
      if (m_collectMode == COLLECT_ACTIVE) {
        setLeds(false, Constants.kCollectLedCount);
      } else if (m_collectMode == COLLECT_DONE) {
        setLeds(true, Constants.kCollectLedCount);
      } else {
        setLeds(false, 0);
      }
      return;
    }
    
    if (m_scoreState == SCORE_LOW) {
      setLeds(m_scoreMode == SCORE_DONE, Constants.kLowLedCount);
    } else if (m_scoreState == SCORE_MED) {
      setLeds(m_scoreMode == SCORE_DONE, Constants.kMedLedCount);
    } else if (m_scoreState == SCORE_HIGH) {
      setLeds(m_scoreMode == SCORE_DONE, Constants.kHighLedCount);
    } else {
      setLeds(false, 0);
    }
  }
  
  private void setLeds(boolean green, int endIdx) {
    if (green) {
      m_candle.setLEDs(m_greenR, m_greenG, m_greenB, 0, 0, endIdx);
      m_candle.setLEDs(0, 0, 0, 0, endIdx, Constants.kLedCount);
    } else {
      m_candle.setLEDs(m_yellowR, m_yellowG, m_yellowB, 0, 0, endIdx);
      m_candle.setLEDs(0, 0, 0, 0, endIdx, Constants.kLedCount);
    }
  }

  public void incrementScoreState() {
    ++m_scoreState;
    if (SCORE_HIGH < m_scoreState) {m_scoreState = SCORE_LOW;}
  }

  public void setScoreActive() {
    m_scoreMode = SCORE_ACTIVE;
  }

  public void setScoreDone() {
    m_scoreMode = SCORE_DONE;
  }

  public void setScoreInactive() {
    m_scoreMode = SCORE_INACTIVE;
  }

  public void setCollectActive() {
    m_collectMode = COLLECT_ACTIVE;
  }

  public void setCollectDone() {
    m_collectMode = COLLECT_DONE;
  }

  public void setCollecInactive() {
    m_collectMode = COLLECT_INACTIVE;
  }

}



/*

Tinman 
                                 ^
                                /|\
                               / | \
                              /  |  \
                           __/___|___\__
                          |            |
                          |     _    | |
                          |    ||\_____|
                          |    \_|_|_|_|
                          |            |
                          |____________|
                        
*/