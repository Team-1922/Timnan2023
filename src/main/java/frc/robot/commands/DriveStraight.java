// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveStraight extends CommandBase {

  DriveTrainSubsystem  m_DriveTrainSubsystem;
 Joystick m_LeftJoystick;
 double Maxrpm = Constants.maxRPM;
 double rightmaxrpm = 2000;

 double deadzone;
  /** Creates a new DriveStraight. */
  public DriveStraight(DriveTrainSubsystem driveTrain, Joystick leftJoystick) {
    m_DriveTrainSubsystem = driveTrain;
    m_LeftJoystick = leftJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    deadzone = SmartDashboard.getNumber("joystick deadzone", 0.125);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
if (m_LeftJoystick.getY() > deadzone || m_LeftJoystick.getY() < deadzone){
    m_DriveTrainSubsystem.velocityDrive(
      (-m_LeftJoystick.getY() * Maxrpm),
      (-m_LeftJoystick.getY() * Maxrpm));
    }

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
