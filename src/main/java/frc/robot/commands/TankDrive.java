// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.atomic.DoubleAccumulator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TankDrive extends CommandBase {
 
   DriveTrainSubsystem m_DriveTrainSubsystem;
   Joystick LeftJoystick;
   Joystick RightJoystick;
   double m_LeftDeadZoneOnOff;
   double m_RightDeadZoneOnOff;
   double JoystickDeadzone;
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


SmartDashboard.putNumber("Deadzone", JoystickDeadzone);

JoystickDeadzone = SmartDashboard.getNumber("Deadzone", 0.125);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ( Math.abs(LeftJoystick.getY()) < JoystickDeadzone) {
     m_LeftDeadZoneOnOff= 0;
    } else {m_LeftDeadZoneOnOff = 1;}
   
    if ( Math.abs(RightJoystick.getY()) < JoystickDeadzone) {
      m_RightDeadZoneOnOff= 0;
     } else {m_RightDeadZoneOnOff = 1;}
    //Something isnt connecting here -- look into why no get input
   m_DriveTrainSubsystem.Drive( LeftJoystick.getRawAxis(1)*m_LeftDeadZoneOnOff,  RightJoystick.getRawAxis(1)*m_RightDeadZoneOnOff);
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
