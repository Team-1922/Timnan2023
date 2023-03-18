// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Apriltag extends CommandBase {
DriveTrainSubsystem m_driveTrain;
;// set this to the tx of the limelight later
Timer timer = new Timer();
Timer failSafe = new Timer();
double turnSpeed;
double startingRotation;
double Turn;
double PGain = Constants.apriltagPGain;
double DGain = Constants.apriltagDGain;
double m_tx;
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); // limelights name goes here 
NetworkTableEntry tx = table.getEntry("tx");
double targetYaw;
  /** Creates a new Apriltag. */
  public Apriltag(DriveTrainSubsystem driveTrain) {
   m_driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    failSafe.reset();
    NetworkTable Ozram = NetworkTableInstance.getDefault().getTable("Ozram");
    PGain = Constants.apriltagPGain; //Ozram.getEntry("visionPGain").getDouble();
    DGain = Constants.apriltagDGain;    // Ozram.getEntry("visionDGain").getDouble(Constants.apriltagDGain);
    NetworkTable gloworm = NetworkTableInstance.getDefault().getTable("limelight");
  //  targetYaw = gloworm.getEntry("targetyaw");

startingRotation = tx.getDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
// get the tx somewhere in execute as well
//m_tx 
failSafe.start();
m_tx = tx.getDouble(0.0);
  turnSpeed = (startingRotation-m_tx)*DGain; 
 
double turn = PGain*m_tx+ turnSpeed;
SmartDashboard.putNumber("turn value", turn);

    

 
m_driveTrain.velocityDrive((turn)*.3*Constants.maxRPM, -(turn)*.3*Constants.maxRPM);
  
if(Math.abs(m_tx) <3.5){timer.start();}
 else{timer.reset();
}
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.Drive(0, 0);
    failSafe.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  { if(timer.get()>.15){return true;}
  if(failSafe.get()>=2.5){return true;}
    
    return false;
  }
}
