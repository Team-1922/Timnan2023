// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TrajectoryDrive extends CommandBase {
  private DriveTrainSubsystem m_driveTrain;

  private Pose2d startingPose;
  private double startingTime;


  private Translation2d endPoseTranslation = new Translation2d(1, 2); 
  private Pose2d endPose = new Pose2d(endPoseTranslation, Rotation2d.fromDegrees(360));
  private ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();

  private TrajectoryConfig config = new TrajectoryConfig(1, 1);

  private Trajectory trajectory;

  private RamseteController ramseteController = new RamseteController();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.distBetweenWheelsMeters);
  private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();
 
  
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

    waypoints.add(new Translation2d(1, 2));
    waypoints.add(new Translation2d(1, 2));
 
    trajectory = TrajectoryGenerator.generateTrajectory(
      startingPose,
      waypoints,
      endPose,
      config
    ); 
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSpeeds = (ramseteController.calculate(m_driveTrain.getRobotPose(), trajectory.sample(System.currentTimeMillis() - startingTime)));
    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

   //  m_driveTrain.velocityDrive(wheelSpeeds.leftMetersPerSecond*metersPerSecondToRPM, wheelSpeeds.rightMetersPerSecond*metersPerSecondToRPM);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getRobotPose() == endPose;
  }
}
