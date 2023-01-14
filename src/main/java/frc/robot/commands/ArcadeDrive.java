// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDrive extends CommandBase {
 DriveTrainSubsystem m_DriveTrainSubsystem;

 Joystick m_LeftJoystick = RobotContainer.LeftJoystick;
   
 Joystick m_RightJoystick = RobotContainer.RightJoystick;
 
 double m_RightY= m_RightJoystick.getY();
 double m_RightX= m_RightJoystick.getX();
 double m_LeftY = m_LeftJoystick.getY();
 double m_LeftX = m_LeftJoystick.getX();
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive() {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 double ArcadedrivePartOne = m_LeftY*m_LeftX;
 double ArcadedrivePartTwo = m_LeftX*m_RightY;
    m_DriveTrainSubsystem.drive(ArcadedrivePartOne, ArcadedrivePartTwo);
//fix that later pls 
    
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
