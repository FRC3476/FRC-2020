package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.OrangeUtility;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;


@SuppressWarnings("unused")
public class NewEightBallOppTrench extends TemplateAuto implements Runnable  {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;


	public NewEightBallOppTrench(double d) {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(120.5 + 15 + 3.5-1, /*-(165-55.5/2)*/  -(106.5-17)/*-131 */));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));

	}

	@Override
	public void run() {

		//Start 120 275
		System.out.println("NewNewEightBallOppTrench");


		
		//Get oponent trench balls
		Path p1 = new Path(here()); //106.5 edge pt,  20 roobt width
		p1.addPoint(new Translation2D(243 - 35.0/2+6 - 40,  -(106.5-17)), 130);

		p1.addPoint(new Translation2D(243 - 35.0/2+6,  -(131)), 130);
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setIntakeState(IntakeState.INTAKE);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
		drive.setAutoPath(p1, true);
		while(!drive.isFinished()) if(isDead()) return;

		shooter.setSpeed(5300);

		//Drive to shooting position abd fire 
		Path p2 = new Path(here());
		p2.addPoint(new Translation2D(125.5, 0), 140);
		p2.addPoint(new Translation2D(120.5, 0), 140);
		drive.setAutoPath(p2, false);
		while(!drive.isFinished()) if(isDead()) return;
		intake.setIntakeState(IntakeState.OFF);
		hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
		shooter.setHoodAngle(33);
		shootBalls(5);
		shooter.setHoodAngle(0);

		//Grab first balls from switch bump
		//94+1205, 12
		turnOnIntakeTrack();


		Path p3 = new Path(here());
		p3.addPoint(new Translation2D(202.5, -1).translateBy(frontBumpDirRight.scale(-15)).translateBy(frontBumpDirLeft.scale(0.0)), 70);
		p3.addPoint(new Translation2D(202.5, -1).translateBy(frontBumpDirRight.scale(0)).translateBy(frontBumpDirLeft.scale(0.0)), 70);
		p3.addPoint(new Translation2D(202.5, -1).translateBy(frontBumpDirRight.scale(60)).translateBy(frontBumpDirLeft.scale(0.0)), 70);
	//was 226 in x,0 in y
		drive.setAutoPath(p3, true);
		while(!drive.isFinished()) if(isDead()) return;
		
		Path p6 = new Path(here());
		p6.addPoint(new Translation2D(202.5, -1).translateBy(frontBumpDirRight.scale(0)).translateBy(frontBumpDirLeft.scale(0.0)), 70);
		p6.addPoint(new Translation2D(125.5, 18/*67*/), 140);
		p6.addPoint(new Translation2D(120.5, 18/*67*/), 140);
		shooter.setSpeed(5300);
		drive.setAutoPath(p6, false);
		while(!drive.isFinished()) if(isDead()) return;

		shooter.setHoodAngle(33);
		shootBalls(5);
		shooter.setHoodAngle(0);
		
 
		synchronized (this) {
			done = true; 
		}


		
	}

}