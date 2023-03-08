// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Apriltag extends CommandBase {
DriveTrainSubsystem m_driveTrain;
double m_tx;// set this to the tx of the limelight later
double turnSpeed;
double startingRotation;
double Turn;
double PGain = Constants.apriltagPGain;
double DGain = Constants.apriltagDGain;
  /** Creates a new Apriltag. */
  public Apriltag(DriveTrainSubsystem driveTrain) {
   m_driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
startingRotation = m_tx;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
// get the tx somewhere in execute as well
  turnSpeed = (startingRotation-m_tx)*DGain; 
 
double turn = PGain*m_tx+ turnSpeed;
    

 
m_driveTrain.velocityDrive(turn*.3, -turn*.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
