// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.atomic.DoubleAccumulator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TankDrive extends CommandBase {
 
   DriveTrainSubsystem m_DriveTrainSubsystem;
   Joystick m_LeftJoystick = RobotContainer.LeftJoystick;
   
   Joystick m_RightJoystick = RobotContainer.RightJoystick;
   double m_LeftX = m_LeftJoystick.getX();
   double m_RightX= m_RightJoystick.getX();
  /** Creates a new TankDrive. */
  public TankDrive(

  ) {
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {





  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    


   //m_DriveTrainSubsystem.drive( m_LeftX,  m_RightX);
   //change the number after the * to adjust the output or whatever 
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
