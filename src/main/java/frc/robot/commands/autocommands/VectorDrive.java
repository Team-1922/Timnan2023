package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class VectorDrive extends CommandBase{
    private DriveTrainSubsystem m_DriveTrain;
    private Timer m_Timer;

    public double[] m_DriveTarget = new double[2];
    public double[] m_IntersectingVector = new double[2];
    public double[] m_DriveDistance = new double[2];
    //these should be relative to the robot's current position
    public double initialPosition;
    public double secondPosition;
    public double deltaPos;
    public boolean check;
    public double midDist;
    public double midTheta;
    public double targetDist;
    public double targetTheta; //find a way to have these as inputs
    public double m_Pitch;
    public double m_Yaw;
    public VectorDrive(DriveTrainSubsystem drive) {
        m_DriveTrain = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_DriveTarget[0] = targetDist;
        m_DriveTarget[1] = targetTheta;
        m_IntersectingVector[0] = midDist;
        m_IntersectingVector[1] = midTheta;
        m_DriveDistance[0] = 0;
        m_DriveDistance[1] = 0;
        m_Timer = new Timer();
    }

    @Override
    public void execute() {
        if (check) {
            //rotate the robot somewhere here
            initialPosition = m_DriveTrain.getLeftEncoderFeet();
            while(!m_Timer.hasElapsed(.1)) {
                secondPosition = m_DriveTrain.getLeftEncoderFeet();
            }
            deltaPos = secondPosition - initialPosition;
            m_Pitch = m_DriveTrain.robotPitch();
            m_Yaw = m_DriveTrain.robotYaw();
            m_DriveDistance[0] += deltaPos;
            m_DriveDistance[1] = 1;
            if(m_DriveDistance[0] == m_IntersectingVector[0]) check = false;
            m_Timer.stop();
            m_Timer.reset();
        }
        else {
            //we'll deal with this later
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 2; i++) {
            m_DriveTarget[i] = 0;
            m_DriveDistance[i] = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return (m_DriveDistance[0] == m_DriveTarget[0] && m_DriveDistance[1] == m_DriveTarget[1]);
    }
}
