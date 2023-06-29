// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmittingDiode;

public class LedCoolAnimation extends CommandBase {
  Animation m_animationOne = new LarsonAnimation(255, 255, 0, 0, .3,46, BounceMode.Back, 2, 8);
  Animation m_animationTwo = new LarsonAnimation(255,255, 0, 0, .3, 46, BounceMode.Back, 2, 54);
 // Animation m_animationThree = new LarsonAnimation(255, 255, 0, 0, .2,23, BounceMode.Front, 2,31);
//  Animation m_animationfour = new LarsonAnimation(255,255, 0, 0, .18, 23, BounceMode.Front, 2, 77);
  LightEmittingDiode m_LED = new LightEmittingDiode();
  /** Creates a new LedCoolAnimation. */
  public LedCoolAnimation(LightEmittingDiode LED) {
m_LED = LED;
addRequirements(LED);
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
    m_LED.LedAnimate(m_animationOne, 1);
    m_LED.LedAnimate(m_animationTwo, 2);
  //  m_LED.LedAnimate(m_animationThree, 3);
  //  m_LED.LedAnimate(m_animationfour, 4);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
