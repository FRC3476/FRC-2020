package frc.auton.guiauto;

import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.AbstractAutonomousStep;
import frc.auton.guiauto.serialization.Autonomous;
import frc.auton.guiauto.serialization.Serializer;
import frc.auton.guiauto.serialization.TrajectoryAutonomousStep;
import frc.subsystem.RobotTracker;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class NetworkAuto extends TemplateAuto {

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("autodata");
    NetworkTableEntry autoPath = table.getEntry("autoPath");
    private Autonomous autonmous = null;
    public NetworkAuto() {
        super(new Translation2D()); // temp
        String autonmousJson = autoPath.getString(null);
        try {
            if(autonmousJson != null){
                autonmous = Serializer.deserialize(autonmousJson);

            } else return;
            
        } catch (ClassNotFoundException | IOException e) {
            e.printStackTrace();
            System.out.print("Got invalid data from networktables");
        }
    }

    @Override
    public void run() {
        for(AbstractAutonomousStep autonomousStep : autonmous.getAutonomousSteps()){
            if(autonomousStep instanceof TrajectoryAutonomousStep){
                TrajectoryAutonomousStep trajectoryAutonomousStep = (TrajectoryAutonomousStep) autonomousStep;
                Trajectory.State intialState = trajectoryAutonomousStep.getStates().get(0);
                RobotTracker.getInstance().setInitialRotation(Rotation2D.fromWPIRotation2d(intialState.poseMeters.getRotation()));
                RobotTracker.getInstance().setInitialTranslation(Translation2D.fromWPITranslation2d(intialState.poseMeters.getTranslation()));
                break;
            }
        }

        for(AbstractAutonomousStep autonomousStep : autonmous.getAutonomousSteps()){
            autonomousStep.execute(this);
        }

        synchronized (this) {
			done = true; 
		}

    }
    
}
