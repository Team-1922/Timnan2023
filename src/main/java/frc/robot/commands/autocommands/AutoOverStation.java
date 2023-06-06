// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoOverStation extends CommandBase {
  private DriveTrainSubsystem m_driveTrain;
  private Timer Clock = new Timer();
  private double m_RPM;
  private double startPitch;
  private double newPitch;

  private boolean check1;
  private boolean check2;

  /** Creates a new AutoStraightBack. */
  public AutoOverStation(DriveTrainSubsystem driveTrain, double RPM) {
    m_driveTrain = driveTrain;
    m_RPM = RPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPitch = m_driveTrain.robotPitch();
    check1 = false;
    check2 = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newPitch = m_driveTrain.robotPitch();

    if (check1) {
      while (Math.abs(newPitch) - startPitch <= 3 && !check2) {
        Clock.start();
        if(!Clock.hasElapsed(.1)) check2 = true;
      }
    }
    else {
    //If the change goes up (Up the ramp)
      if(newPitch - startPitch >= 3 && !check1){
        check1 = true;
      }
    }
    m_driveTrain.velocityDrive(m_RPM, m_RPM);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (check1 && check2);
  }
}
