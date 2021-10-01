package frc.auton.guiauto;

import java.io.File;
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

public abstract class AbstractGuiAuto extends TemplateAuto {

    private  Autonomous autonmous;
    public AbstractGuiAuto(File autonmousFile) {
        super(new Translation2D()); // unused
        
        try {
            autonmous = Serializer.deserializeFromFile(autonmousFile);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.print("Got invalid data the file");
        }
    }

    @Override
    public void run() {
        if(autonmous != null){
            System.out.println("Running: " + this.getClass().getName());
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
        } else {
            System.out.println("auto is null");
        }

        synchronized (this) {
			done = true; 
		}

    }
    
}
