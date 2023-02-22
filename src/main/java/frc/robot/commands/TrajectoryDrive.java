// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TrajectoryDrive extends CommandBase {
  private DriveTrainSubsystem m_driveTrain;
 // private PhotonCamera limelight = new PhotonCamera("gloworm");
 // private PhotonPipelineResult result;

  private Pose2d startingPose;
  private Timer timer = new Timer();

  private Translation2d endPoseTranslation;
  private Transform2d endTransform;
  private Pose2d endPose;

  private ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();

  private TrajectoryConfig config = new TrajectoryConfig(((Constants.maxRPM/5)*Constants.metersPerSecondToRPM), (Constants.maxRPM*Constants.metersPerSecondToRPM)/4);

  private Trajectory trajectory;

  private RamseteController ramseteController = new RamseteController();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.distBetweenWheelsMeters);
  private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

  private Trajectory.State trajectoryState;
 
  
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
    timer.start();

    endPoseTranslation = new Translation2d(1, 1);
    endTransform = new Transform2d(endPoseTranslation, Rotation2d.fromDegrees(0));
    endPose = startingPose.plus(endTransform);
    // Note start and end pose on smartdashboard
    //teddy stuff thinks it starts at 5, 5


    waypoints.add(new Translation2d(-1, 0));
    waypoints.add(new Translation2d(-1, 0));
 
    trajectory = TrajectoryGenerator.generateTrajectory(
      startingPose,
      waypoints,
      endPose,
      config
    ); 

    m_driveTrain.setTrajectory(trajectory);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 //   result = limelight.getLatestResult();

    trajectoryState = trajectory.sample(timer.get());
    
    chassisSpeeds = (ramseteController.calculate(m_driveTrain.getRobotPose(), trajectoryState));
    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);


    m_driveTrain.velocityDrive(wheelSpeeds.leftMetersPerSecond/Constants.metersPerSecondToRPM, wheelSpeeds.rightMetersPerSecond/Constants.metersPerSecondToRPM);


    double test = trajectoryState.poseMeters.getY();
    SmartDashboard.putNumber("TrajectoryTest", test);
    SmartDashboard.putNumber("TrajecWheelDifference", (wheelSpeeds.leftMetersPerSecond/Constants.metersPerSecondToRPM - wheelSpeeds.rightMetersPerSecond/Constants.metersPerSecondToRPM));
    // Code thinks at any given time the pose Should be the endPose value
    // That means the code IMMEDIATELY THINKS it should be done
    // which I don't know how to fix but I think that's our problem

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
