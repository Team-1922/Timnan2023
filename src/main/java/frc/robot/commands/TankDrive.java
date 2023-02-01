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
  
  /** Creates a new TankDrive. */
  public TankDrive(DriveTrainSubsystem driveTrain

  ) {
  m_DriveTrainSubsystem = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {





  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    //Something isnt connecting here -- look into why no get input
   m_DriveTrainSubsystem.Drive( m_LeftJoystick.getY(),  m_RightJoystick.getY());
   //change the number after the * to adjust the output or whatever 
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
