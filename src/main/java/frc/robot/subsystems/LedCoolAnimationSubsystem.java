// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LedCoolAnimation;

public class LedCoolAnimationSubsystem extends SubsystemBase {
  LightEmittingDiode m_LightEmittingDiode = new LightEmittingDiode();
  LedCoolAnimation m_LedCoolAnimation = new LedCoolAnimation(m_LightEmittingDiode);
  
  /** Creates a new LedCoolAnimation. */
  public LedCoolAnimationSubsystem(LedCoolAnimation LedCoolAnimation, LightEmittingDiode lightEmittingDiode) {
    
LedCoolAnimation = m_LedCoolAnimation;
  }



  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public void CoolAnimation(LedCoolAnimation m_LedCoolAnimation, LightEmittingDiode m_LightEmittingDiode){
  ///m_LightEmittingDiode
    m_LedCoolAnimation.initialize();
  m_LedCoolAnimation.execute();
  }
  // Called when the command is initially scheduled.

}
