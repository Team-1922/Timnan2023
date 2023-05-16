// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TankDrive extends CommandBase {
 
   DriveTrainSubsystem m_DriveTrainSubsystem;
   Joystick LeftJoystick;
   Joystick RightJoystick;
   double m_LeftDeadZoneOnOff;
   double m_RightDeadZoneOnOff;
   double deadzone;
   
  /** Creates a new TankDrive. */
  public TankDrive(DriveTrainSubsystem m_driveTrain, Joystick m_LeftJoystick, Joystick m_RightJoystick

  ) {
  m_DriveTrainSubsystem = m_driveTrain;
     LeftJoystick = m_LeftJoystick;
     RightJoystick = m_RightJoystick;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


deadzone = 0.05;
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ( Math.abs(LeftJoystick.getY()) < deadzone) {
     m_LeftDeadZoneOnOff= 0;
    } else {m_LeftDeadZoneOnOff = 1;}
   
    if ( Math.abs(RightJoystick.getY()) < deadzone) {
      m_RightDeadZoneOnOff= 0;
     } else {m_RightDeadZoneOnOff = 1;}


   m_DriveTrainSubsystem.Drive( Math.pow(-LeftJoystick.getY()*.5, 3)*m_LeftDeadZoneOnOff,  Math.pow(-RightJoystick.getY()*.5, 3)*m_RightDeadZoneOnOff);

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
