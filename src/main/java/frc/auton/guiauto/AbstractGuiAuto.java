package frc.auton.guiauto;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
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
            DriverStation.reportError("Auotonmous Failed to deserialize", e.getStackTrace());
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
            DriverStation.reportError("auto is null", true);
        }

        synchronized (this) {
			done = true; 
		}

    }
    
}
