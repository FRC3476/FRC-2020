package frc.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;
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
public class OpponentSteal extends TemplateAuto implements Runnable  {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;

	Trajectory trajectory1;
	Trajectory trajectory2;


	public OpponentSteal() {
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(60), Units.inchesToMeters(30.42928));
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(20)));

		ArrayList<Pose2d> path1 = new ArrayList<>();
		path1.add(new Pose2d(3.58, -3.65, Rotation2d.fromDegrees(180)));
		path1.add(new Pose2d(6.17, -3.65, Rotation2d.fromDegrees(180)));

		trajectoryConfig.setReversed(true);
		trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

		ArrayList<Pose2d> path2 = new ArrayList<>();
		path1.add(new Pose2d(6.17, -3.65, Rotation2d.fromDegrees(180)));
		path2.add(new Pose2d(3.05, 1.05, Rotation2d.fromDegrees(165)));
		
		trajectoryConfig.setReversed(false);
		trajectory2 = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);


	}

	public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

	@Override
	public void run() {
		Pose2d initalPose = new Pose2d(3.58, -3.65, Rotation2d.fromDegrees(180));

		robotTracker.setInitialTranslation(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
		robotTracker.setInitialRotation(Rotation2D.fromWPIRotation2d(initalPose.getRotation()));
		System.out.println("Oponent Steal");


		turnOnIntakeTrack();
		drive.setAutoPath(trajectory1);
		while(!drive.isFinished()) if(isDead()) return;

		System.out.println("here1");
		turnOffIntakeTrack();

		shooter.setSpeed(4000); //May need to set hood pos

		drive.setAutoPath(trajectory2);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println("here2");

		shootBalls(5);

		// drive.setAutoPath(trajectory3);
		// while(!drive.isFinished()) if(isDead()) return;
		// System.out.println("here3");

		// drive.setAutoPath(trajectory4);
		// while(!drive.isFinished()) if(isDead()) return;
		// System.out.println("here4");
		
 
		synchronized (this) {
			done = true; 
		}


		
	}

}