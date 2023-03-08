// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmittingDiode;

public class AnimateStop extends CommandBase {
  /** Creates a new LightUpGreen. */
  private final LightEmittingDiode m_LED;

  public AnimateStop(LightEmittingDiode LED) {
   m_LED = LED;
 
   addRequirements(m_LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LED.LedAnimate(null, 0);
m_LED.LedAnimate(null, 1);
m_LED.LedAnimate(null, 2);


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
