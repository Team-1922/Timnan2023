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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimation extends SubsystemBase {
DriveTrainSubsystem m_DriveTrain = new DriveTrainSubsystem(null);
private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.distBetweenWheelsMeters);  
private double loopCount = 0;

private NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("limelight");
private DoubleArraySubscriber botPose = cameraTable.getDoubleArrayTopic("botpose").subscribe(new double[]{});


  /** Creates a new Pose_estimation. */
  public PoseEstimation() {}

  
private final DifferentialDrivePoseEstimator m_PoseEstimator = new DifferentialDrivePoseEstimator(
  kinematics, 
  Rotation2d.fromDegrees(m_DriveTrain.robotYaw()), 
  m_DriveTrain.getLeftEncoderFeet()*Constants.feetToMeters, 
  m_DriveTrain.getRightEncoderFeet()*Constants.feetToMeters, 
  m_DriveTrain.getRobotPose(), 
  VecBuilder.fill(0.05,0.05,Units.degreesToRadians(5)), 
  VecBuilder.fill(0.5,0.5,Units.degreesToRadians(30)));

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    loopCount++;
    m_PoseEstimator.update(Rotation2d.fromDegrees(m_DriveTrain.robotYaw()), m_DriveTrain.getLeftEncoderFeet()*Constants.feetToMeters, m_DriveTrain.getRightEncoderFeet()*Constants.feetToMeters);

    if(loopCount >= 5){
      double[] pose = new double[7];

      pose = botPose.get();
      Pose3d pose3d = new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));



      m_PoseEstimator.addVisionMeasurement(pose3d.toPose2d(), Timer.getFPGATimestamp() - (pose[6]/1000));
      loopCount = 0;
    }

  }


  // Use this for any 'get pose' calls
  public Pose2d EstimatePose(){
    return m_PoseEstimator.getEstimatedPosition();
  }
}
