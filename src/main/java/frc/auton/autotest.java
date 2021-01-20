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
public class autotest extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;


	public autotest(double startY) {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(0, 0));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
	}

	@Override
	public void run() {

		//Start 75 120
		System.out.println("Only Shoot");
		double turnAngle = 0;
		
		//turnAngle = Math.toDegrees(Math.atan2(75 , here().getX()-48));
		Translation2D target = new Translation2D(0, 67);
		Translation2D robot = here();

		Path p1 = new Path(here());
		p1.addPoint(new Translation2D(0, 100), 50);
		p1.addPoint(new Translation2D(50, 150), 50);
		p1.addPoint(new Translation2D(50, 300), 50);
		drive.setAutoPath(p1, false);
		
		while(!drive.isFinished()) if(isDead()) return;

		synchronized (this) {
			done = true; 
		}

	}

}
