// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import frc.robot.RobotContainer;
import frc.robot.commands.Apriltag;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GatherTheCube;
import frc.robot.commands.Score;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightEmittingDiode;
import frc.robot.subsystems.ScoreMode;
import frc.robot.commands.autocommands.AutoOverStation;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));

  }

  private static DriveTrainSubsystem m_driveTrain = RobotContainer.m_DriveTrainSubsystem;
  private static ScoreMode m_scoreMode = RobotContainer.m_ScoreMode;
  private static Arm m_arm = RobotContainer.m_Arm;
  private static EndEffector m_endEffector = RobotContainer.m_EndEffector;
  private static LightEmittingDiode m_LED = RobotContainer.m_LightEmittingDiode;





  private static AutoStraight m_autoStraight = new AutoStraight(m_driveTrain, 3000);
  private static AutoStraightBack m_autoStraightBack = new AutoStraightBack(m_driveTrain, -3000);
  private static AutoStraightBalance m_autoStraightBalance = new AutoStraightBalance(m_driveTrain, 3500);
  private static final AutoOverStation m_overStation = new AutoOverStation(m_driveTrain, 3000, 200);

 private static AutoTimerDrive m_timerDrive = new AutoTimerDrive(m_driveTrain, 2.25);



  private static final AutoBalance AutoBalance(){

    AutoBalance m_autoBalance = new AutoBalance(m_driveTrain);
    return m_autoBalance;
  }

  private static final AutoSetMode SetMode(int mode){

    AutoSetMode m_setMode = new AutoSetMode(m_scoreMode, m_LED, mode);
    return m_setMode;
  }

  private static final Score Score(){
    Score m_score = new Score(m_arm, m_endEffector, m_scoreMode);
    return m_score;
  }


  private static final TrajectoryDrive CubeTrajectory(boolean reversed, double left){
// pass 'left' -1 for right side
    Translation2d waypoint1 = new Translation2d(.25, 0);
    Translation2d waypoint2 = new Translation2d(1.5, (left * .01));
    Translation2d waypoint3 = new Translation2d(3, left * .2);
    Pose2d endPose = new Pose2d(new Translation2d(5.7, left * 0), Rotation2d.fromDegrees(0));


    TrajectoryDrive m_trajectory = new TrajectoryDrive(m_driveTrain, waypoint1, waypoint2, waypoint3, endPose, reversed);

    return m_trajectory;
  }

  private static final TrajectoryDrive HomeTrajectory(boolean reversed, double left){

    Translation2d waypoint1 = new Translation2d(3,  left * 0);
    Translation2d waypoint2 = new Translation2d(2, 0);
    Translation2d waypoint3 = new Translation2d(1.5, 0);
    Pose2d endPose = new Pose2d(new Translation2d(.15, left * -.1), Rotation2d.fromDegrees(0));


    TrajectoryDrive m_trajectory = new TrajectoryDrive(m_driveTrain, waypoint1, waypoint2, waypoint3, endPose, reversed);

    return m_trajectory;
  }


  private static final SetBrake Brake(){
    SetBrake m_brake = new SetBrake(m_driveTrain);
    return m_brake;
  }

  private static final SetCoast Coast(){
    SetCoast m_coast = new SetCoast(m_driveTrain);
    return m_coast;
  }

  private static final Apriltag Aim(){
    Apriltag m_aim = new Apriltag(m_driveTrain);
    return m_aim;
  }

  private static final GatherTheCube Gather(){
    GatherTheCube m_gather = new GatherTheCube(m_arm, m_endEffector);
    return m_gather;
  }




  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static final SequentialCommandGroup m_autoStraightGroup = new SequentialCommandGroup(SetMode(3), Score(), m_autoStraight, m_autoStraightBack, AutoBalance());
  public static final SequentialCommandGroup m_autoBackup = new SequentialCommandGroup(SetMode(3), Score(), m_timerDrive);
  public static final SequentialCommandGroup m_autoStraightToBalance = new SequentialCommandGroup(SetMode(3), Score(), m_autoStraightBalance, AutoBalance());
  public static final SequentialCommandGroup m_test = new SequentialCommandGroup(m_overStation);



  public static final SequentialCommandGroup m_trajectoryAutoLEFT = new SequentialCommandGroup(
    SetMode(1), 
    Score(), 
    new ParallelDeadlineGroup(Gather(), CubeTrajectory(false, 1)), 
    Brake(),  
    SetMode(3), 
    HomeTrajectory(true, 1), 
    Coast(),
    Aim(),
    Score());



    public static final SequentialCommandGroup m_trajectoryAutoRIGHT = new SequentialCommandGroup(
    SetMode(1), 
    Score(), 
    new ParallelDeadlineGroup(Gather(), CubeTrajectory(false, -1)), 
    Brake(),  
    SetMode(3), 
    HomeTrajectory(true, -1), 
    Coast(),
    Aim(),
    Score());

}
