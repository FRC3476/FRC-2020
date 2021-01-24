package frc.auton;

import frc.subsystem.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;

import frc.subsystem.Intake;

public class GalacticSearchB extends TemplateAuto {
    Drive drive = Drive.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    Intake intake = Intake.getInstance();
    VisionManager vision = VisionManager.getInstance();
    Shooter shooter = Shooter.getInstance();

    boolean killSwitch = false;

    public GalacticSearchB() {
        super(new Translation2D(15, 60));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
    }

    public void run(){
        Path bluePath = new Path(here());

        bluePath.addPoint(new Translation2D(180,60), 30);
        bluePath.addPoint(new Translation2D(240,120), 30);
        bluePath.addPoint(new Translation2D(300,60), 30);
        bluePath.addPoint(new Translation2D(345,30), 30);

        drive.setAutoPath(bluePath, false);

        while(!drive.isFinished()) {
            if(isDead()) {
                return;
            }
        }

		synchronized (this) {
			done = true; 
		}
    }
}
