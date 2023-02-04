package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightEmitingDiode;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Random;

/** An example command that uses an example subsystem. */
public class ScoreLedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LightEmitingDiode m_ledSubsystem;

  private int m_counter;
  private int m_minCount = 100;
  private int m_maxCount = 100; // 400
  private boolean m_done;

  private Random m_random;

  /**
   * Creates a new ScoreLedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoreLedCommand(LightEmitingDiode ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_counter = 0;
    m_done = false;
    m_ledSubsystem.setScoreActive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_done) {return;}

    ++m_counter;
    if ((m_minCount <= m_counter
         && m_random.nextInt(0, m_maxCount-m_minCount+1) == 0)
        || m_maxCount < m_counter) {
      m_done = true;
      m_ledSubsystem.setScoreDone();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledSubsystem.setScoreInactive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
