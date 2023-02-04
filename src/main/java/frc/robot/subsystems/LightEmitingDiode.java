// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LightEmitingDiode extends SubsystemBase {
  private int SCORE_LOW = 1;
  private int SCORE_MED = 2;
  private int SCORE_HIGH = 3;

  private int SCORE_INACTIVE = 0;
  private int SCORE_ACTIVE = 1;
  private int SCORE_DONE = 2;
  
  private int m_scoreState;
  private int m_scoreMode;
  
  private int COLLECT_INACTIVE = 0;
  private int COLLECT_ACTIVE = 1;
  private int COLLECT_DONE = 2;

  private int m_collectMode;

  private final CANdle m_candle = new CANdle(Constants.kCandleId, "rio");

  private final int m_yellowR = 255;
  private final int m_yellowG = 255;
  private final int m_yellowB = 0;
  private final int m_greenR = 0;
  private final int m_greenG = 255;
  private final int m_greenB = 0;

  private int m_counter;

  // private Animation m_animation = new RainbowAnimation();
  /** Creates a new LED. */
  public LightEmitingDiode() {
    // CANdleConfiguration configAll = new CANdleConfiguration();
    // configAll.statusLedOffWhenActive = false;
    // configAll.disableWhenLOS = false;
    // configAll.stripType = LEDStripType.GRB;
    // configAll.brightnessScalar = 0.1;
    // configAll.vBatOutputMode = VBatOutputMode.Modulated;
    // m_candle.configAllSettings(configAll, 100);

    // m_candle.configFactoryDefault();

    m_counter = 0;

    reset();
    // we can either use the leds as a status indicator, or as just a decoration, both if we can figure that out in our spare line.

  }

  @Override
  public void periodic() {
    // ++m_counter;
    SmartDashboard.putNumber("led counter", m_counter);
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
      setLeds(m_scoreMode == SCORE_DONE, Constants.kCollectLedCount + Constants.kLowLedCount);
    } else if (m_scoreState == SCORE_MED) {
      setLeds(m_scoreMode == SCORE_DONE, Constants.kCollectLedCount + Constants.kLowLedCount + Constants.kMedLedCount);
    } else if (m_scoreState == SCORE_HIGH) {
      setLeds(m_scoreMode == SCORE_DONE, Constants.kCollectLedCount + Constants.kLowLedCount + Constants.kMedLedCount + Constants.kHighLedCount);
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

  public void reset() {
    m_scoreState = SCORE_LOW;
    m_scoreMode = SCORE_INACTIVE;
    m_collectMode = COLLECT_INACTIVE;

    SmartDashboard.putNumber("LED collect mode", m_collectMode);
    SmartDashboard.putNumber("LED score mode", m_scoreMode);
    SmartDashboard.putNumber("LED score state", m_scoreState);

    m_candle.clearAnimation(0);
  }

  public void incrementScoreState() {
    ++m_counter;
    ++m_scoreState;
    if (SCORE_HIGH < m_scoreState) {m_scoreState = SCORE_LOW;}
    SmartDashboard.putNumber("LED score state", m_scoreState);
  }

  public void setScoreActive() {
    ++m_counter;
    m_scoreMode = SCORE_ACTIVE;
    SmartDashboard.putNumber("LED score mode", m_scoreMode);
  }

  public void setScoreDone() {
    ++m_counter;
    m_scoreMode = SCORE_DONE;
    SmartDashboard.putNumber("LED score mode", m_scoreMode);
  }

  public void setScoreInactive() {
    ++m_counter;
    m_scoreMode = SCORE_INACTIVE;
    SmartDashboard.putNumber("LED score mode", m_scoreMode);
  }

  public void setCollectActive() {
    ++m_counter;
    m_collectMode = COLLECT_ACTIVE;
    SmartDashboard.putNumber("LED collect mode", m_collectMode);
  }

  public void setCollectDone() {
    ++m_counter;
    m_collectMode = COLLECT_DONE;
    SmartDashboard.putNumber("LED collect mode", m_collectMode);
  }

  public void setCollecInactive() {
    ++m_counter;
    m_collectMode = COLLECT_INACTIVE;
    SmartDashboard.putNumber("LED collect mode", m_collectMode);
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