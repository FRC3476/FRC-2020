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
        super(new Translation2D(45, 60));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
    }

    public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

    public void run(){
        Path bluePath = new Path(here());

        bluePath.addPoint(new Translation2D(180,60), 30);
        bluePath.addPoint(new Translation2D(240,120), 30);
        bluePath.addPoint(new Translation2D(300,60), 30);
        bluePath.addPoint(new Translation2D(345,30), 30);

        turnOnIntakeTrack();
        drive.setAutoPath(bluePath, true);

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