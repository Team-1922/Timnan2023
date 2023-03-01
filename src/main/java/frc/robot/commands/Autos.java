// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private static DriveTrainSubsystem m_driveTrain = RobotContainer.m_DriveTrainSubsystem;

  private static AutoStraight m_autoStraight = new AutoStraight(m_driveTrain, 3000);
  private static AutoStraightBack m_autoStraightBack = new AutoStraightBack(m_driveTrain, -2700);
  private static AutoBalance m_autoBalance = new AutoBalance(m_driveTrain);

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


  public static final SequentialCommandGroup m_autoStraightGroup = new SequentialCommandGroup(m_autoStraight, m_autoStraightBack, m_autoBalance);
}
