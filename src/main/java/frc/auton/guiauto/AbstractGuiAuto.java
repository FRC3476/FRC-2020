package frc.auton.guiauto;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
        try {
            autonmous = Serializer.deserializeFromFile(autonmousFile);
        } catch (IOException e) {
            DriverStation.reportError("Failed to deserialize auto", e.getStackTrace());
        }
    }

    public AbstractGuiAuto(String autonomousJson) {
        try {
            autonmous = Serializer.deserialize(autonomousJson);
        } catch (IOException | ClassNotFoundException e) {
            DriverStation.reportError("Failed to deserialize auto", e.getStackTrace());
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

    
    Rotation2D initalRotation2d = new Rotation2D();
    Translation2D initalTranslation2d = new Translation2D();

    @Override
    public void run() {
        System.out.println("Started Running: " + Timer.getFPGATimestamp());
        RobotTracker.getInstance().setInitialRotation(initalRotation2d);
        RobotTracker.getInstance().setInitialTranslation(initalTranslation2d);

        for(AbstractAutonomousStep autonomousStep : autonmous.getAutonomousSteps()){
            System.out.println("doing a step: " + Timer.getFPGATimestamp());
            autonomousStep.execute(this);
        }

        System.out.println("finished: " + Timer.getFPGATimestamp());

        synchronized (this) {
			done = true; 
		}

    }
    
}
