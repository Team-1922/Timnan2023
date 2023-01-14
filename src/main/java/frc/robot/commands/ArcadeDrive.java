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
 double m_LeftX = m_LeftJoystick.getX();
 double m_RightY= m_RightJoystick.getY();
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

   // m_DriveTrainSubsystem.drive(m_LeftX, m_RightY*2);
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
