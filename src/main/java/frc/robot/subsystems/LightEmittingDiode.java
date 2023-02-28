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


public class LightEmittingDiode extends SubsystemBase {

  
  private final CANdle m_candle = new CANdle(Constants.kCandleId);


  // private Animation m_animation = new RainbowAnimation();
  /** Creates a new LED. */
  public LightEmittingDiode() {
 

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

  
 }
 
  
public void setColor(int red, int green, int blue){
m_candle.setLEDs(red, green, blue);

}
public void LedAnimate(Animation Animation){
  m_candle.animate(Animation);
  
} }





/*

Tinman 
                                     
                                     /|\
                                   /  |  \
                                 /    |    \
                           __/___|___\__
                          |                         |
                          |     _    | |          |
                          |    ||\_____|    |
                          |    \_|_|_|_|     | 
                          |                         |
                          |____________|
                        
*/