// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrainSubsystem;

public class XBoxTankDrive extends CommandBase {
  /** Creates a new XBoxTankDrive. */

  private DriveTrainSubsystem m_driveTrain;
  private CommandXboxController m_xbox;

  public XBoxTankDrive(DriveTrainSubsystem driveTrain, CommandXboxController xbox) {
    m_driveTrain = driveTrain;
    m_xbox = xbox;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.Drive(-m_xbox.getLeftY()*.35, -m_xbox.getRightY()*.35);
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
