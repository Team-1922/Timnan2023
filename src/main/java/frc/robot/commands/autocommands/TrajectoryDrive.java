// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import java.util.ArrayList;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
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
  private Translation2d m_waypoint1;
  private Translation2d m_waypoint2;
  private Translation2d m_waypoint3;
  private Pose2d m_endingPose;

 // private PhotonCamera limelight = new PhotonCamera("gloworm");
 // private PhotonPipelineResult result;

  private Pose2d startingPose;
  private Timer timer = new Timer();

  private Translation2d endPoseTranslation;
  private Transform2d endTransform;
  private Pose2d endPose;

  private ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();


  private Trajectory trajectory;

  private RamseteController ramseteController = new RamseteController();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.distBetweenWheelsMeters);
  private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

  private Trajectory.State trajectoryState;
 
  private TrajectoryConfig config = new TrajectoryConfig(((Constants.maxRPM*Constants.metersPerSecondToRPM)/8), (Constants.maxRPM*Constants.metersPerSecondToRPM)).setKinematics(kinematics);




  /** Creates a new TrajectoryDrive. */
  public TrajectoryDrive(DriveTrainSubsystem driveTrain, Translation2d waypoint1, Translation2d waypoint2, Translation2d waypoint3, Pose2d endingPose, boolean reversed) {
    m_driveTrain = driveTrain;
    m_waypoint1 = waypoint1;
    m_waypoint2 = waypoint2;
    m_waypoint3 = waypoint3;

    m_endingPose = endingPose;

    config.setReversed(reversed);
    config.setEndVelocity(250*Constants.metersPerSecondToRPM); // Adjust to go faster

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPose = m_driveTrain.getRobotPose();
    timer.start();
    timer.reset(); //Incase the command doesn't end and stop timer

  /*  
    endPoseTranslation = new Translation2d(0, 2);
    endTransform = new Transform2d(endPoseTranslation, Rotation2d.fromDegrees(-90));
    //endPose = startingPose.plus(endTransform);
    endPose = new Pose2d(endPoseTranslation, Rotation2d.fromDegrees(180));
  */

    waypoints.clear();

    waypoints.add(m_waypoint1);
    waypoints.add(m_waypoint2);  
    waypoints.add(m_waypoint3);

  //***NOTE***//
  // All mid-points are generated relative to the (0, 0) made on startup, 
  // even in a new run of the command.

  // So then if I had constant waypoints and just kept
  // repeating TrajectoryDrive, it would find its way back 
  // to the same spots. (But from a NEW startingPose, interesting)
 
    trajectory = TrajectoryGenerator.generateTrajectory(
      startingPose,
      waypoints,
      m_endingPose,
      config
    ); 

    m_driveTrain.setTrajectory(trajectory);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 //   result = limelight.getLatestResult();

    trajectoryState = trajectory.sample(timer.get());
    SmartDashboard.putNumber("State X", trajectoryState.poseMeters.getX());
    
    chassisSpeeds = (ramseteController.calculate(m_driveTrain.getRobotPose(), trajectoryState));
    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);


    m_driveTrain.velocityDrive((wheelSpeeds.leftMetersPerSecond/Constants.metersPerSecondToRPM)*1.2, (wheelSpeeds.rightMetersPerSecond/Constants.metersPerSecondToRPM)*1.2);


    double test = 1;
    SmartDashboard.putNumber("TrajectoryTest", test);
    SmartDashboard.putNumber("TrajecWheelDifference", (wheelSpeeds.leftMetersPerSecond/Constants.metersPerSecondToRPM));

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
    return Math.abs(m_driveTrain.getRobotPose().getX() - m_endingPose.getX() ) <.1
        && Math.abs(m_driveTrain.getRobotPose().getY() - m_endingPose.getY() ) <.1;  
 
    }
}
