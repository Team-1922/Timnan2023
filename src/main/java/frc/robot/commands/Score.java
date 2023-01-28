// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Score extends CommandBase {
  EndEffector cubePositioner = new EndEffector();
  private Arm robotArm = new Arm();
  private int scoreMode = EndEffector.m_ScoreMode;
  /** Creates a new Score. */
  public Score() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotArm.setNewFF();
    if (scoreMode == 1) {robotArm.setAngle(Constants.kPivotMotorLowAngle);
    } else if (scoreMode == 2) {robotArm.setAngle(Constants.kPivotMotorMidAngle);
    } else if (scoreMode == 3) {robotArm.setAngle(Constants.kPivotMotorHighAngle);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cubePositioner.getHasObject();
  }
}
