package frc.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.math.*;
import frc.utility.Limelight;
import frc.utility.control.motion.Path;

import frc.subsystem.Intake;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;

@SuppressWarnings("unused")
public class GalacticSearchA extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;

    private boolean pathIsRed;

    public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

	public void turnOffIntakeTrack() {
        intake.setDeployState(Intake.DeployState.UNDEPLOY);
		intake.setSpeed(0);
		hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
	}
    
	public GalacticSearchA() {
        //RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
        super(new Translation2D(0, 0));
        int startY;

        pathIsRed = Limelight.getInstance().isTargetVisiable();
        startY = pathIsRed ? 90 : 30;
        
		RobotTracker.getInstance().setInitialTranslation(new Translation2D(46, startY));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
	}

	@Override
	public void run() {
        int speed = pathIsRed ? 60 : 70;

        Path p1 = new Path(here());
        if(pathIsRed) {
            p1.addPoint(new Translation2D(90, 90), speed);
            p1.addPoint(new Translation2D(150, 60), speed);
            p1.addPoint(new Translation2D(180, 150), speed);
            p1.addPoint(new Translation2D(314, 150), speed);
        }else {
            p1.addPoint(new Translation2D(180, 30), speed);
            p1.addPoint(new Translation2D(210, 120), speed);
            p1.addPoint(new Translation2D(270, 90), speed);
            p1.addPoint(new Translation2D(314, 90), speed);
        }
        
        turnOnIntakeTrack();

        drive.setAutoPath(p1, true);
        
        while(!drive.isFinished()) {
            if(p1.getPercentage() > 0.95) {
                turnOffIntakeTrack();
            }

            if(isDead()) {
                return;
            }
        }

		synchronized (this) {
			done = true; 
		}
	}

}
