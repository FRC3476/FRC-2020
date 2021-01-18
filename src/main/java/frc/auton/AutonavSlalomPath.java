package frc.auton;

import java.util.ArrayList;

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
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
	}

	@Override
	public void run() {
        ArrayList<Translation2D> points = new ArrayList<Translation2D>();
        points.add(new Translation2D(90, 30));
        points.add(new Translation2D(90, 90));
        points.add(new Translation2D(270, 90));
        points.add(new Translation2D(270, 30));
        points.add(new Translation2D(330, 30));
        points.add(new Translation2D(330, 90));
        points.add(new Translation2D(270, 90));
        points.add(new Translation2D(270, 30));
        points.add(new Translation2D(90, 30));
        points.add(new Translation2D(90, 90));
        points.add(new Translation2D(42, 90));

        Path p1 = new Path(here());
        for(int i = 0; i < 11; i++) {
            p1.addPoint(points.get(i), 20);
        }

        drive.setAutoPath(p1, false);

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
