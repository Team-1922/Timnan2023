package frc.robot.commands.autocommands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.vision;

public class TrackDistanceAuto extends CommandBase{
    double angleOffset;
    public double initalPosition;
    public double currentPosition;
    public double deltaPos;
    public double forwardDistance;
    public double overallDistance;
    public double m_Yaw;
    public double deltaYaw;
    //stuff for tracking distance
    public boolean m_ApriltagDetected;
    private Timer m_Timer;
    private static DriveTrainSubsystem m_DriveTrain;
    private static vision m_Camera;


    public TrackDistanceAuto(DriveTrainSubsystem drive, vision camera) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry m_tx = table.getEntry("tx");

        m_DriveTrain = drive;
        m_Camera = camera;

        addRequirements(drive);
        addRequirements(camera);
    }
    
    @Override
    public void initialize() {
        forwardDistance = 0;
        angleOffset = 0;
        SmartDashboard.getNumber("overallDistance", overallDistance);
        SmartDashboard.getNumber("Angle", angleOffset);
        SmartDashboard.getBoolean("ApriltagFound", m_ApriltagDetected);
        m_Timer = new Timer();
    }

    @Override
    public void execute() {
        m_ApriltagDetected = SmartDashboard.getBoolean("ApriltagFound", m_ApriltagDetected);
        if (m_ApriltagDetected) {
            m_Camera.calculateApriltagOffset(angleOffset);
            forwardDistance = (m_Camera.detectApriltag(forwardDistance)*Math.cos(angleOffset));
        } else {
            //tracking distance stuff
            m_Yaw = (m_DriveTrain.robotYaw() % 360);
            initalPosition = m_DriveTrain.getLeftEncoderFeet();
            m_Timer.start();
            while(!m_Timer.hasElapsed(.05)) {
                currentPosition = m_DriveTrain.getLeftEncoderFeet();
                deltaYaw = (m_DriveTrain.robotYaw() % 360) - m_Yaw;
                deltaPos = currentPosition - initalPosition;
            }
            forwardDistance += deltaPos*Math.cos(deltaYaw);
            angleOffset += deltaYaw;
            m_Timer.stop();
            m_Timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("overallDistance", overallDistance);
        SmartDashboard.putNumber("Angle", angleOffset);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}