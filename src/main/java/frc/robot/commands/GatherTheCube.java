// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GatherTheCube extends CommandBase {
  private Arm m_RobotArm;
  private EndEffector m_CubeHarvester;
  /** Creates a new GatherTheCube. */
  public GatherTheCube(Arm pivotArm, EndEffector cubeEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RobotArm = pivotArm;
    m_CubeHarvester = cubeEffector;

    addRequirements(pivotArm);
    addRequirements(cubeEffector);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RobotArm.setNewFF();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RobotArm.setNewFF();
    m_RobotArm.setAngle(Constants.kPivotMotorGatherAngle);
    m_CubeHarvester.gatherTheCube();
    while (!EndEffector.m_hasObject) {
      //sensor stuff
      EndEffector.m_hasObject = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_RobotArm.setAngle(Constants.kPivotMotorStowAngle);
    return m_CubeHarvester.getHasObject();
  }
}
