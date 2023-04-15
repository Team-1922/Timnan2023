// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LedCoolAnimation;

public class CoolLedSubsystem extends SubsystemBase {
LightEmittingDiode m_LightEmittingDiode = new LightEmittingDiode();
LedCoolAnimation m_LedCoolAnimation = new LedCoolAnimation(m_LightEmittingDiode);
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
 
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
