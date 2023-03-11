// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class FlipTankDrive extends CommandBase {
 
   DriveTrainSubsystem m_driveTrain;
   Joystick LeftJoystick;
   Joystick RightJoystick;
   double m_LeftDeadZoneOnOff;
   double m_RightDeadZoneOnOff;
   double deadzone;

   boolean flipped;
   
  /** Creates a new TankDrive. */
  public FlipTankDrive(DriveTrainSubsystem driveTrain, Joystick m_LeftJoystick, Joystick m_RightJoystick

  ) {
  m_driveTrain = driveTrain;
     LeftJoystick = m_LeftJoystick;
     RightJoystick = m_RightJoystick;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


deadzone = 0.125;
 
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

     
    flipped = m_driveTrain.getFlipped();


   m_driveTrain.flipDrive( Math.pow(-LeftJoystick.getY()*.75*Constants.maxRPM, 3)*m_LeftDeadZoneOnOff,  Math.pow(-RightJoystick.getY()*.75*Constants.maxRPM, 3)*m_RightDeadZoneOnOff, flipped);

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
