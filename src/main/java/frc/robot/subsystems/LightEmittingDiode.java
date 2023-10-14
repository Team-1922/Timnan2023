// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;


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
m_candle.clearAnimation(0);
m_candle.clearAnimation(1);
m_candle.clearAnimation(2);
m_candle.clearAnimation(3);
m_candle.clearAnimation(4);
m_candle.setLEDs(red, green, blue,0,0,108);


}
public void LedAnimate(Animation Animation, int AnimationSlot){
 
  m_candle.animate(Animation, AnimationSlot);
  
} }





/*

Tinman 

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