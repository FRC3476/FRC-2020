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
public class GalacticSearchA extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;


	public GalacticSearchA() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(46, 90));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
	}

	@Override
	public void run() {
        //need some vision stuff
        ArrayList<Translation2D> points / new ArrayList<Translation2D>;
        if() {
            points.add(new Translation2D(90, 90));
            points.add(new Translation2D(150, 60));
            points.add(new Translation2D(180, 150));
            points.add(new Translation2D(314, 150));
        }else {
            points.add(new Translation2D(180, 30));
            points.add(new Translation2D(210, 120));
            points.add(new Translation2D(270, 90));
            points.add(new Translation2D(314, 90));
        }
        
        Path p1 = new Path();
        

		synchronized (this) {
			done = true; 
		}
	}

}
