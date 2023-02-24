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

  private TrajectoryConfig config = new TrajectoryConfig(((Constants.maxRPM/3)*Constants.metersPerSecondToRPM), (Constants.maxRPM*Constants.metersPerSecondToRPM)/2);

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

    endPoseTranslation = new Translation2d(1, 3);
    endTransform = new Transform2d(endPoseTranslation, Rotation2d.fromDegrees(-90));
    endPose = startingPose.plus(endTransform);

    // Note start and end pose on smartdashboard
    SmartDashboard.putNumber("StartPoseX", startingPose.getX());
    SmartDashboard.putNumber("StartPoseY", startingPose.getY());
    SmartDashboard.putNumber("EndPoseX", endPose.getX());
    SmartDashboard.putNumber("EndPoseY", endPose.getY());


    waypoints.add(new Translation2d(.5, 1));
    waypoints.add(new Translation2d(.99, 2.99));

  //***NOTE***//
  // All mid-points are generated relative to the (0, 0) made on startup, 
  // even in a new instance of the command.

  // So then if I had constant waypoints and just kept
  // repeating TrajectoryDrive, it would find its way back 
  // to the same spots. (But from a NEW startingPose, interesting)
 
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


    m_driveTrain.velocityDrive((wheelSpeeds.leftMetersPerSecond/Constants.metersPerSecondToRPM), (wheelSpeeds.rightMetersPerSecond/Constants.metersPerSecondToRPM));


    double test = wheelSpeeds.leftMetersPerSecond/Constants.metersPerSecondToRPM;
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
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getRobotPose().getX() - endPose.getX() ) <.2
     && Math.abs(m_driveTrain.getRobotPose().getY() - endPose.getY() ) <.2 ;  }
}
