package frc.auton;

import frc.subsystem.*;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;

@SuppressWarnings("unused")
public class TemplateAuto implements Runnable { 
    Drive drive = Drive.getInstance();
    //CollisionManager collisionManager = CollisionManager.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    
    int side = 1;

    boolean killSwitch = false;
    boolean done = false; 
    //Translation 2D is in inches.

    public TemplateAuto(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
    }

    public TemplateAuto(Translation2D start, int side) {
        RobotTracker.getInstance().setInitialTranslation(start);
        this.side = side;
    }

    public Translation2D here() {
        return RobotTracker.getInstance().getOdometry().translationMat;
    }
    
    public Rotation2D dir() {
        return RobotTracker.getInstance().getOdometry().rotationMat;
    }

    synchronized public void killSwitch() {
        killSwitch = true;
    }

    synchronized public boolean isDead() {
        return killSwitch;
    }

    synchronized public boolean isFinished() {
        return done; 
    }

    @Override
    public void run() {

    }

}
