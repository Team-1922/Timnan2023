// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GatherTheCube extends CommandBase {
  private Arm m_Arm;
  private EndEffector m_EndEffector;
  private Timer timer = new Timer();
  private Timer cubeTimer = new Timer();
  /** Creates a new GatherTheCube. */
  public GatherTheCube(Arm pivotArm, EndEffector cubeEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = pivotArm;
    m_EndEffector = cubeEffector;

    addRequirements(pivotArm);
    addRequirements(cubeEffector);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.setAngle(Constants.kPivotMotorGatherAngle);
    cubeTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_EndEffector.gatherTheCube();

    if(m_EndEffector.hasCube()){
      timer.start();
    } else {
      timer.reset();
    }
    cubeTimer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Timer.delay(.2);
    //May use start command if delay causes problems.
      // Delay causes problems. 
    m_EndEffector.stopMotors();
    m_Arm.setAngle(Constants.kPivotMotorLowAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false; 
    //if(m_Arm.hasCube()== true) {cubeTimer.start();}
    return cubeTimer.hasElapsed(.75);

  }
}
