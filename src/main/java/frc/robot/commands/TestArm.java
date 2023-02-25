package frc.robot.commands;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestArm extends CommandBase{
    
  private Arm m_Arm;
  public TestArm(Arm Arm) {
    m_Arm = Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.setAngle(300);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setAngle(
      Constants.kPivotMotorLowAngle
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
