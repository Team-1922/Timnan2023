// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmittingDiode;

public class LedAmericaAnimation extends CommandBase {
  // don't question the name
  /** Creates a new LightUpGreen. */
  private final LightEmittingDiode m_LED;
  
  Animation m_PartOne = new SingleFadeAnimation(255,0,0,0,0.3,60);
  Animation m_PartTwo = new SingleFadeAnimation(255,255,255,255,0.25,60);
  Animation m_PartThree = new SingleFadeAnimation(0,0,255,0, 0.25,60);
  Timer m_LedTImer = new Timer();
  public LedAmericaAnimation(LightEmittingDiode LED) {
   m_LED = LED;

   addRequirements(m_LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LED.LedAnimate(null, 0);
    m_LED.LedAnimate(null, 1);
    m_LED.LedAnimate(null, 2);
    m_LedTImer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
if( m_LedTImer.get()<4){
m_LED.LedAnimate(m_PartOne,  1);}
if(  m_LedTImer.get() >4 &&m_LedTImer.get() <7.5) {   

  m_LED.LedAnimate(m_PartTwo, 1);}
  if(m_LedTImer.get() >7.5 &&m_LedTImer.get() <12) {   
 
    m_LED.LedAnimate(m_PartThree, 1);}
if ( m_LedTImer.hasElapsed(15.5)){m_LedTImer.reset();}


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
