// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import frc.robot.RobotContainer;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GatherTheCube;
import frc.robot.commands.Score;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightEmittingDiode;
import frc.robot.subsystems.ScoreMode;
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
  private static AutoStraightBalance m_autoStraightBalance = new AutoStraightBalance(m_driveTrain, 3000);

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


  private static final TrajectoryDrive CubeTrajectory(boolean reversed){

    Translation2d waypoint1 = new Translation2d(.5, 0);
    Translation2d waypoint2 = new Translation2d(1, 0);
    Translation2d waypoint3 = new Translation2d(3, .01);
    Pose2d endPose = new Pose2d(new Translation2d(2.9, 0), Rotation2d.fromDegrees(0));


    TrajectoryDrive m_trajectory = new TrajectoryDrive(m_driveTrain, waypoint1, waypoint2, waypoint3, endPose, reversed);

    return m_trajectory;
  }

  private static final TrajectoryDrive HomeTrajectory(boolean reversed){

    Translation2d waypoint1 = new Translation2d(2, 0);
    Translation2d waypoint2 = new Translation2d(1, 0);
    Translation2d waypoint3 = new Translation2d(0.1, 0);
    Pose2d endPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));


    TrajectoryDrive m_trajectory = new TrajectoryDrive(m_driveTrain, waypoint1, waypoint2, waypoint3, endPose, reversed);

    return m_trajectory;
  }






  private static AutoBalance m_autoBalance = new AutoBalance(m_driveTrain);
  private static AutoBalance m_autoBalance2 = new AutoBalance(m_driveTrain);


  private static AutoSetMode m_setMode1 = new AutoSetMode(m_scoreMode, m_LED, 3);
  private static AutoSetMode m_setMode2 = new AutoSetMode(m_scoreMode, m_LED, 3);
  private static AutoSetMode m_setMode3 = new AutoSetMode(m_scoreMode, m_LED, 3);

  private static Score m_score2 = new Score(m_arm, m_endEffector, m_scoreMode);
  private static Score m_score3 = new Score(m_arm, m_endEffector, m_scoreMode);


  private static GatherTheCube m_gather = new GatherTheCube(m_arm, m_endEffector);
  private static SetBrake m_brake = new SetBrake(m_driveTrain);
 



  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static final SequentialCommandGroup m_autoStraightGroup = new SequentialCommandGroup(SetMode(3), Score(), m_autoStraight, m_autoStraightBack, AutoBalance());
  public static final SequentialCommandGroup m_autoBackup = new SequentialCommandGroup(SetMode(3), Score(), m_timerDrive);
  public static final SequentialCommandGroup m_autoStraightToBalance = new SequentialCommandGroup(SetMode(3), Score(), m_autoStraightBalance, AutoBalance());
  public static final SequentialCommandGroup m_trajectoryAuto = new SequentialCommandGroup(
    SetMode(3), 
    Score(), 
    new WaitCommand(.25), 
    new ParallelDeadlineGroup(m_gather, CubeTrajectory(false)), 
    m_brake, 
    HomeTrajectory(true), 
    SetMode(2), 
    Score());

}
