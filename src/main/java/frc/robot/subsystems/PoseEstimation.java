// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimation extends SubsystemBase {
DriveTrainSubsystem m_DriveTrain = new DriveTrainSubsystem(null);
  
  private Pigeon2 m_Gyro = new Pigeon2(Constants.kPigeon); 
  


  /** Creates a new Pose_estimation. */
  public PoseEstimation() {}

  
private final DifferentialDrivePoseEstimator m_PoseEstimator =
 new DifferentialDrivePoseEstimator(
   
  null, null, m_DriveTrain.getLeftEncoderFeet(),
 
 m_DriveTrain.getRightEncoderFeet(), m_DriveTrain.getRobotPose(),
 VecBuilder.fill(0.05,0.05,Units.degreesToRadians(5)),
 VecBuilder.fill(0.5,0.5,Units.degreesToRadians(30))
 );
// m_PoseEstimator.update(null,m_DriveTrain.getLeftEncoderFeet(), m_DriveTrain.getRightEncoderFeet());


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
