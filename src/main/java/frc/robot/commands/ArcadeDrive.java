// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDrive extends CommandBase {
  RobotContainer m_RobotContainer;

 
  DriveTrainSubsystem m_DriveTrainSubsystem;
  Joystick leftJoystick;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrainSubsystem m_DriveTrain, Joystick m_leftJoystick) {
    m_DriveTrainSubsystem  = m_DriveTrain;
  leftJoystick = m_leftJoystick;
    addRequirements(m_DriveTrain);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DifferentialDrive.curvatureDriveIK(leftJoystick.getX(), leftJoystick.getZ(), true);
    
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
