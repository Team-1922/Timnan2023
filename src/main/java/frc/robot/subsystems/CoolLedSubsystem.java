// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.LedCoolAnimation;

public class CoolLedSubsystem extends SubsystemBase {
LightEmittingDiode m_LightEmittingDiode = new LightEmittingDiode();
LedCoolAnimation m_LedCoolAnimation = new LedCoolAnimation(m_LightEmittingDiode);
Animation RainbowAnimation = new com.ctre.phoenix.led.RainbowAnimation(1,0.5,108);
private TimeOfFlight m_TOF = new TimeOfFlight(Constants.kFrontSensorID);

  /** Creates a new CoolLedSubsystem. */
  public CoolLedSubsystem( LightEmittingDiode lightEmittingDiode, LedCoolAnimation ledCoolAnimation) {
    m_LightEmittingDiode = lightEmittingDiode;
    m_LedCoolAnimation = ledCoolAnimation;
  }


public void DisabledAnimation( ){
  m_LedCoolAnimation.initialize();
  m_LedCoolAnimation.execute();
 }

 public void clearAnimation( ){
  m_LedCoolAnimation.initialize();
 
 }// this is to test if the TOF thinks it sees a cube
 public void Cube(){
  if(m_TOF.getRange() <= 160 && m_TOF.getRange() >= 50){
  m_LightEmittingDiode.setColor(160, 32, 240);}
  if (m_TOF.getRange() >= 160 || m_TOF.getRange() <= 50  ){
   
m_LightEmittingDiode.LedAnimate(RainbowAnimation, 0);
  }

 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
