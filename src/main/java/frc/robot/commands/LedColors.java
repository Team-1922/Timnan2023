// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightEmittingDiode;

public class LedColors extends CommandBase {
  /** Creates a new LightUpGreen. */
  private final LightEmittingDiode m_LED;
  int m_green;
  int m_blue;
  int m_red;
  public LedColors(LightEmittingDiode LED, int red, int green, int blue) {
   m_LED = LED;
   m_blue = blue;
   m_green = green;
   m_red = red;
   addRequirements(m_LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_LED.setColor(m_red,m_green,m_blue);

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
