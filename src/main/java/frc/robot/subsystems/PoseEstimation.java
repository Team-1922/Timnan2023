// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.linsol.qr.AdjLinearSolverQr_FDRM;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PoseEstimation extends SubsystemBase {
public final static Field2d m_Field2d = new Field2d();
private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.distBetweenWheelsMeters);  
private double loopCount = 0;
private DriveTrainSubsystem m_DriveTrain = RobotContainer.m_DriveTrainSubsystem;

private NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("limelight");
private DoubleArraySubscriber botPose = cameraTable.getDoubleArrayTopic("botpose").subscribe(new double[]{});
private Pose3d lastPose = visionPose();




  /** Creates a new Pose_estimation. */
  public PoseEstimation() {
  SmartDashboard.putData(m_Field2d);

  }

  
private final DifferentialDrivePoseEstimator m_PoseEstimator = new DifferentialDrivePoseEstimator(
  kinematics, 
  Rotation2d.fromDegrees(m_DriveTrain.robotYaw()), 

  -m_DriveTrain.getRightEncoderFeet()*Constants.feetToMeters, 
  -m_DriveTrain.getLeftEncoderFeet()*Constants.feetToMeters, 
    // Note on 'forward' in relation to apriltag
  // Forward will move the robot closer to x0 ideally, on either red or blue side
  // the actual positive and negative change does not really line up with this, it will 
  // need different signage depending on color.
    // flipping (left and right) to (-right and -left) swaps 'forward'


  visionPose().toPose2d(), 
  VecBuilder.fill(.1,.1,Units.degreesToRadians(10)), // tune these tune these
  VecBuilder.fill(.5,.5,Units.degreesToRadians(10))); // pretty sure these are x,y,rotation values
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    loopCount++;
    m_PoseEstimator.update(Rotation2d.fromDegrees(m_DriveTrain.robotYaw()), -m_DriveTrain.getRightEncoderFeet()*Constants.feetToMeters, -m_DriveTrain.getLeftEncoderFeet()*Constants.feetToMeters);



    if(loopCount >= 5){
      if(Math.abs(visionPose().getX() - lastPose.getX()) <= 1 && Math.abs(visionPose().getY() - lastPose.getY()) <= 1 && visionPose().getX() != 0){
        m_PoseEstimator.addVisionMeasurement(visionPose().toPose2d(), Timer.getFPGATimestamp() - (poseTime()/1000));
        loopCount = 0;
        System.out.print("vision added");

      }
      
    }

    m_Field2d.setRobotPose(EstimatePose());
    SmartDashboard.putNumber("bose X", EstimatePose().getX());
    lastPose = visionPose();

  }


  public Pose3d visionPose(){
    double[] pose = new double[7];

    pose = botPose.get();
    Pose3d pose3d = new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));

    return pose3d;
  }

  public double poseTime(){
    double[] pose = new double[7];

    pose = botPose.get();
    return pose[6];
  }

  // Use this for any 'get pose' calls
  public Pose2d EstimatePose(){
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void setTrajectory(Trajectory traj){
    m_Field2d.getObject("traj").setTrajectory(traj);
  }
}
