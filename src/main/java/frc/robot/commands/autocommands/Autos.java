// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import frc.robot.RobotContainer;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Score;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightEmittingDiode;
import frc.robot.subsystems.ScoreMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private static AutoStraightBack m_autoStraightBack2 = new AutoStraightBack(m_driveTrain, 3000);
  private static AutoStraightBalance m_autoStraightBalance = new AutoStraightBalance(m_driveTrain, 3000);

  private static AutoBalance m_autoBalance = new AutoBalance(m_driveTrain);
  private static AutoBalance m_autoBalance2 = new AutoBalance(m_driveTrain);

  private static AutoSetMode m_setMode1 = new AutoSetMode(m_scoreMode, m_LED, 3);
  private static AutoSetMode m_setMode2 = new AutoSetMode(m_scoreMode, m_LED, 3);
  private static AutoSetMode m_setMode3 = new AutoSetMode(m_scoreMode, m_LED, 3);


  private static Score m_score = new Score(m_arm, m_endEffector, m_scoreMode);

  private static AutoTimerDrive m_timerDrive = new AutoTimerDrive(m_driveTrain, 2.25);
  private static Score m_score2 = new Score(m_arm, m_endEffector, m_scoreMode);
  private static Score m_score3 = new Score(m_arm, m_endEffector, m_scoreMode);


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
  public static final SequentialCommandGroup m_autoDriveTest = new SequentialCommandGroup(m_autoStraightBack);
  //public static final SequentialCommandGroup m_autoStraightGroup = new SequentialCommandGroup(SetMode(3), Score(), m_autoStraight, m_autoStraightBack, AutoBalance());
  //public static final SequentialCommandGroup m_autoBackup = new SequentialCommandGroup(SetMode(3), Score(), m_timerDrive);
  //public static final SequentialCommandGroup m_autoStraightToBalance = new SequentialCommandGroup(SetMode(3), Score(), m_autoStraightBalance, AutoBalance());
  /*public static final SequentialCommandGroup m_trajectoryAuto = new SequentialCommandGroup(
    SetMode(2), 
    Score(), 
  //  new WaitCommand(.25), 
    new ParallelDeadlineGroup(m_gather, CubeTrajectory(false)), 
    m_brake,  
    SetMode(3), 
    HomeTrajectory(true), 
    m_aim,
    Score());
    */

}
