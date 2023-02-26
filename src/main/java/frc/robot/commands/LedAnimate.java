// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmitingDiode;

public class LedAnimate extends CommandBase {
  /** Creates a new LightUpGreen. */
  private final LightEmitingDiode m_LED;
  private final Animation m_Animation;
  public LedAnimate(LightEmitingDiode LED, Animation Animation) {
   m_LED = LED;
   m_Animation = Animation;
   addRequirements(m_LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_LED.LedAnimate(m_Animation);


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
