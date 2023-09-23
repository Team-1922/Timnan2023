// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrainSubsystem;

public class XboxCurvyDrive extends CommandBase {
  DriveTrainSubsystem m_driveTrain;
 
  boolean deadzone;
  CommandXboxController m_XboxController;

  /** Creates a new CurvyDrive. */
  public XboxCurvyDrive(DriveTrainSubsystem driveTrain, CommandXboxController xboxController) {
    m_driveTrain = driveTrain;
  
    m_XboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_XboxController.getLeftY()) >= .1){
      deadzone = true;
    }

    m_driveTrain.curvatureDrive(-m_XboxController.getRawAxis(1), -m_XboxController.getRawAxis(4)*.9 , deadzone);
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
