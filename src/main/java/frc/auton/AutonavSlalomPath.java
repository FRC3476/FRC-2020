package frc.auton;

import java.util.LinkedHashMap;
import java.util.Map;

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
        Map<Translation2D, Integer> lhm = new LinkedHashMap<>();
        lhm.put(new Translation2D(90, 30), 30);
        lhm.put(new Translation2D(90, 90), 50);
        lhm.put(new Translation2D(270, 90), 30);
        lhm.put(new Translation2D(270, 30), 30);
        lhm.put(new Translation2D(318, 30), 15);
        lhm.put(new Translation2D(318, 90), 30);
        lhm.put(new Translation2D(270, 90), 30);
        lhm.put(new Translation2D(270, 60), 30);
        lhm.put(new Translation2D(270, 30), 50);
        lhm.put(new Translation2D(90, 30), 30);
        lhm.put(new Translation2D(90, 90), 30);
        lhm.put(new Translation2D(42, 87), 30);

        Path p1 = new Path(here());
        for(Map.Entry<Translation2D, Integer> me : lhm.entrySet()) {
            p1.addPoint(me.getKey(), me.getValue());
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
