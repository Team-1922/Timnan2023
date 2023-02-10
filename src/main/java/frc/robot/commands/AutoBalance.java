// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PIDCommand {
  /** Creates a new AutoBalance. */
  private DriveTrainSubsystem m_driveTrain; 




  public AutoBalance(DriveTrainSubsystem driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(SmartDashboard.getNumber("Balance P", .015), 0, SmartDashboard.getNumber("Balance D", .01)), // Need to insert proper pid values here, waiting until testing
        // This should return the measurement
        () -> driveTrain.robotPitch() + .8,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          SmartDashboard.putNumber("output", output);
          driveTrain.Drive(-output, -output); // Pitch down is pos, wheels need to go same sign as pitch
        });

        m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(.5); //this is the angular range (deg) that it is okay stopping in, also waiting until testing
    getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //getController().atSetpoint(); 
  }
}
