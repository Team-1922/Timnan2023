package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.vision;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DynamicScore extends CommandBase{
    private Arm m_Arm;
    private EndEffector m_EndEffector;
    private vision m_Camera;
    private DriveTrainSubsystem m_DriveTrain;

    /* Creates a new DynamicScore */
    public DynamicScore(Arm Arm, EndEffector endEffector, vision Camera, DriveTrainSubsystem Wheels) {
        m_Arm = new Arm();
        m_EndEffector = new EndEffector();
        m_Camera = new vision();
        m_DriveTrain = new DriveTrainSubsystem(null);

        addRequirements(Arm);
        addRequirements(endEffector);
        addRequirements(Camera);
        addRequirements(Wheels);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
