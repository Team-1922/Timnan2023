// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Apriltagcommand extends CommandBase {
DriveTrainSubsystem m_driveTrain;

boolean hasTarget;
double targetYaw;


//put cameras name in line below
PhotonCamera camera = new PhotonCamera("camera");


  /** Creates a new Apriltag. */
  public Apriltagcommand(DriveTrainSubsystem driveTrain) {
   m_driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     var result = camera.getLatestResult();
    hasTarget = result.hasTargets();


 PhotonTrackedTarget target = result.getBestTarget();
 double targetYaw = target.getYaw();

while(hasTarget == true && Math.abs(targetYaw)>50){

m_driveTrain.velocityDrive(
  targetYaw/300 *Constants.maxRPM, -targetYaw/300*Constants.maxRPM
  );

}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if(hasTarget == false || hasTarget == true && targetYaw <50)
    {
    return true;
  }else{return false;}

  }

}