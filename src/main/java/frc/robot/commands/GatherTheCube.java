// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GatherTheCube extends CommandBase {
  EndEffector cubeHarvester = new EndEffector();
  private Arm robotArm = new Arm();
  /** Creates a new GatherTheCube. */
  private GatherTheCube() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotArm.setNewFF();
    if (EndEffector.m_ScoreMode == 0) robotArm.setAngle(Constants.kPivotMotorGatherAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cubeHarvester.getHasObject();
  }
}
