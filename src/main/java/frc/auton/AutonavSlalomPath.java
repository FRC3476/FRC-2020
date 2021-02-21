package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;

import frc.subsystem.Intake;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;

@SuppressWarnings("unused")
public class AutonavSlalomPath extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;


	public AutonavSlalomPath() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(42, 30));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
	}

	@Override
	public void run() {
        int speed = 75;

        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(60, 30), speed);
        p1.addPoint(new Translation2D(105, 90), speed);
        p1.addPoint(new Translation2D(250, 90), speed);
        p1.addPoint(new Translation2D(280, 10), speed);
        p1.addPoint(new Translation2D(320, 80), speed);
        p1.addPoint(new Translation2D(290, 70), speed);
        
        // drive.setAutoPath(p1, false);

        // while(!drive.isFinished()) if(isDead()) return;

        // Path p2 = new Path(here());

        p1.addPoint(new Translation2D(270, 40), speed);
        p1.addPoint(new Translation2D(250, 20), speed);
        p1.addPoint(new Translation2D(110, 25), speed);
        p1.addPoint(new Translation2D(80, 85), speed);
        p1.addPoint(new Translation2D(42, 90), speed);

        drive.setAutoPath(p1, false);

        while(!drive.isFinished()) if(isDead()) return;

		synchronized (this) {
			done = true;
		}
		
	}

}
