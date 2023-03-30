// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoWAIT extends CommandBase {
 double m_seconds;
private Timer timer = new Timer(); 
private boolean endCommand = false;
DriveTrainSubsystem m_DriveTrain;
  /** Creates a new WaitCommand. */
  public AutoWAIT(DriveTrainSubsystem driveTrain,Double Seconds) {
    m_seconds = Seconds;
    m_DriveTrain = driveTrain;
    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_DriveTrain.Drive(0,0);
  
if(timer.get() >=m_seconds){
  endCommand = true;
}else{endCommand= false;}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
