package frc.auton.guiauto;

import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
    String autonmousJson;
    public NetworkAuto() {
        autonmousJson = autoPath.getString(null);
    }

    Rotation2D initalRotation2d = new Rotation2D();
    Translation2D initalTranslation2d = new Translation2D();
    
    public void init(){
        try {
            if(autonmousJson != null){
                autonmous = Serializer.deserialize(autonmousJson);

            } else return;
            
        } catch (ClassNotFoundException | IOException e) {
            e.printStackTrace();
            System.out.println("Got invalid data from networktables");
        }

        for(AbstractAutonomousStep autonomousStep : autonmous.getAutonomousSteps()){
            if(autonomousStep instanceof TrajectoryAutonomousStep){
                TrajectoryAutonomousStep trajectoryAutonomousStep = (TrajectoryAutonomousStep) autonomousStep;
                Trajectory.State intialState = trajectoryAutonomousStep.getStates().get(0);
                initalRotation2d = Rotation2D.fromWPIRotation2d(intialState.poseMeters.getRotation());
                initalTranslation2d = Translation2D.fromWPITranslation2d(intialState.poseMeters.getTranslation());
                break;
            }
        }
    }

    @Override
    public void run() {
        System.out.println("Started Running: " + Timer.getFPGATimestamp());
        RobotTracker.getInstance().setInitialRotation(initalRotation2d);
        RobotTracker.getInstance().setInitialTranslation(initalTranslation2d);
        if(autonmous != null){
            System.out.println("Started Running2: " + Timer.getFPGATimestamp());

            for(AbstractAutonomousStep autonomousStep : autonmous.getAutonomousSteps()){
                System.out.println("doing a step: " + Timer.getFPGATimestamp());
                autonomousStep.execute(this);
            }

            System.out.println("finished: " + Timer.getFPGATimestamp());
        } else {
            System.out.println("auto is null");
        }

        synchronized (this) {
			done = true; 
		}

    }
    
}
