package frc.robot.subsystems;
import javax.swing.Action;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectPlacementSubsystem extends SubsystemBase{

    boolean hasObject = true;
    public CommandBase PlaceObject() {
        hasObject = false;
        return runOnce(null);
    }

    @Override
    public void periodic() {

    }
}
