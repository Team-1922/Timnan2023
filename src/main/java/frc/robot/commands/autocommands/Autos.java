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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
 // final SendableChooser
  }

  private static DriveTrainSubsystem m_driveTrain = RobotContainer.m_DriveTrainSubsystem;
  private static ScoreMode m_scoreMode = RobotContainer.m_ScoreMode;
  private static Arm m_arm = RobotContainer.m_Arm;
  private static EndEffector m_endEffector = RobotContainer.m_EndEffector;
  private static LightEmittingDiode m_LED = RobotContainer.m_LightEmittingDiode;

  private static AutoStraight m_autoStraight = new AutoStraight(m_driveTrain, 3000);
  private static AutoStraightBack m_autoStraightBack = new AutoStraightBack(m_driveTrain, -2700);
  private static AutoStraightBack m_autoStraightBack2 = new AutoStraightBack(m_driveTrain, 3000);

  private static AutoBalance m_autoBalance = new AutoBalance(m_driveTrain);
  private static AutoSetMode m_setMode1 = new AutoSetMode(m_scoreMode, 3);
  private static AutoSetMode m_setMode2 = new AutoSetMode(m_scoreMode, 3);

  private static Score m_score = new Score(m_arm, m_endEffector, m_scoreMode, m_LED);

  private static AutoTimerDrive m_timerDrive = new AutoTimerDrive(m_driveTrain, 2.25);
  private static Score m_score2 = new Score(m_arm, m_endEffector, m_scoreMode, m_LED);

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static final SequentialCommandGroup m_autoStraightGroup = new SequentialCommandGroup(m_setMode1, m_score, m_autoStraight, m_autoStraightBack, m_autoBalance);
 // public static final SequentialCommandGroup m_autoStraightHalf = new SequentialCommandGroup(m_setMode1, m_score, m_autoStraightBack2, m_autoBalance);

  public static final SequentialCommandGroup m_autoBackup = new SequentialCommandGroup(m_setMode2, m_score2, m_timerDrive);
}
