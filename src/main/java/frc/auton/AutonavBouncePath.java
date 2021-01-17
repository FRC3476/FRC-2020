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
public class AutonavBouncePath extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;


	public AutonavBouncePath() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(42, 90));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
	}

	@Override
	public void run() {
        Translation2D point1 = new Translation2D(60, 90);
        Translation2D point2 = new Translation2D(90, 150);
        
		Translation2D robot = here();

        Path p1 = new Path(here());
        p1.addPoint(point1, 20);
        p1.addPoint(point2, 20);
        drive.setAutoPath(p1, false);

        while(!drive.isFinished()) if(isDead()) return;

        synchronized(this){
            done = true;
        }
	}

}
