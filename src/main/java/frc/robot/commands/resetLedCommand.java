package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.LightEmitingDiode;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ResetLedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LightEmitingDiode m_ledSubsystem;

  /**
   * Creates a new resetLedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ResetLedCommand(LightEmitingDiode ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledSubsystem.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
