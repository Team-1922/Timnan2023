// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmittingDiode;

public class LedAmericaAnimation extends CommandBase {
  // don't question the name
  /** Creates a new LightUpGreen. */
  private final LightEmittingDiode m_LED;
  private final Animation m_Animation;
  int m_AnimationSlot; 
  public LedAmericaAnimation(LightEmittingDiode LED, Animation Animation, int AnimationSlot) {
   m_LED = LED;
   m_Animation = Animation;
   m_AnimationSlot = AnimationSlot;
   addRequirements(m_LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LED.LedAnimate(null, 0);
    m_LED.LedAnimate(null, 1);
    m_LED.LedAnimate(null, 2);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_LED.LedAnimate(m_Animation, 1);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
