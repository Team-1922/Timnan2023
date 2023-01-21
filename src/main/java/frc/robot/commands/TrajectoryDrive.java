// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TrajectoryDrive extends CommandBase {
  private DriveTrainSubsystem m_driveTrain;

  private Pose2d startingPose;
  private double startingTime;


  private Translation2d endPoseTranslation = new Translation2d(1, 2); 
  private Pose2d endPose = new Pose2d(endPoseTranslation, Rotation2d.fromDegrees(360));
  
  /** Creates a new TrajectoryDrive. */
  public TrajectoryDrive(DriveTrainSubsystem driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPose = m_driveTrain.getRobotPose();
    startingTime = System.currentTimeMillis();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
