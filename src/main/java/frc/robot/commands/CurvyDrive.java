// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CurvyDrive extends CommandBase {
  DriveTrainSubsystem m_driveTrain;
  Joystick m_left;
  Joystick m_right;
  boolean deadzone;

  /** Creates a new CurvyDrive. */
  public CurvyDrive(DriveTrainSubsystem driveTrain, Joystick left, Joystick right) {
    m_driveTrain = driveTrain;
    m_left = left;
    m_right = right;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_left.getY()) >= .1){
      deadzone = true;
    }

    m_driveTrain.curvatureDrive(-m_left.getY(), -m_right.getX(), deadzone);
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
