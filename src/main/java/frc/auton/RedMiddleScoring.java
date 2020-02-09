package frc.auton; //red

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.Shooter;
import frc.subsystem.Shooter.ShooterState;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;


public class RedMiddleScoring implements Runnable { 
    Drive drive = Drive.getInstance();
    Elevator elevator = Elevator.getInstance();
    CollisionManager collisionManager = CollisionManager.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    
    int side = 1;

    boolean killSwitch = false;
    //Translation 2D is in inches.  

    public RedMiddleScoring(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
    }

    public RedMiddleScoring(Translation2D start, int side) {
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
    double ShooterFinishTime = 0;
    int stage = 0;
    @Override
    public void run() {

    //Shooter.aim Wait for Varun so Import is avalible

        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(75, -194), 60); //Random Speed, most likely need to change points.
        p1.addPoint(new Translation2D(20, -75), 60); //Random Speed, almost sure need to change points.

        drive.setAutoPath(p1, false);

        //Shooter.setDesired(this.side*-100, true); Need to talk to Varun about this to create one solid system for angling shooter.
        //Shooter.setHeight()
        while(!drive.isFinished()) {
            if(isDead()) return;
            switch(stage) {
                case 0:
                    }
            }
        }
    }
}
