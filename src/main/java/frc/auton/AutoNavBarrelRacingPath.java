package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
public class AutoNavBarrelRacingPath extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;


	public AutoNavBarrelRacingPath() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(42,90));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
	}

	@Override
	public void run() {
		Path p1 = new Path(here());
		p1.addPoint(new Translation2D(150, 90), 60);
		p1.addPoint(new Translation2D(170,60), 60);
		p1.addPoint(new Translation2D(150,40), 60);
		p1.addPoint(new Translation2D(145,70), 60);
		p1.addPoint(new Translation2D(200,60), 60);
		p1.addPoint(new Translation2D(240,75), 60);
		p1.addPoint(new Translation2D(280,125), 60);
		p1.addPoint(new Translation2D(250,140), 60);
		p1.addPoint(new Translation2D(200,120), 60);
		p1.addPoint(new Translation2D(240,75), 60);
		p1.addPoint(new Translation2D(280,40), 60);
		p1.addPoint(new Translation2D(300,35), 60);
		p1.addPoint(new Translation2D(320,65), 60);
		p1.addPoint(new Translation2D(280,75), 60);
		p1.addPoint(new Translation2D(165,85), 60);
		p1.addPoint(new Translation2D(40,90), 60);
		drive.setAutoPath(p1, false);
		
		while(!drive.isFinished()) if(isDead()) return;

		synchronized (this) {
			done = true; 
		}

	}

}
