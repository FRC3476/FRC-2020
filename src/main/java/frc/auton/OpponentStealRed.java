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
public class OpponentStealRed extends TemplateAuto implements Runnable  {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;

	Trajectory trajectory1;
	Trajectory trajectory2;


	public OpponentStealRed() {
		drive.configRamsete(2, 0.7);
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(100), Units.inchesToMeters(80));
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(25)));

		ArrayList<Pose2d> path1 = new ArrayList<>();
		path1.add(new Pose2d(3.6533, -2.92, Rotation2d.fromDegrees(180)));
		//path1.add(new Pose2d(6.0837, -3.332, Rotation2d.fromDegrees(120)));
		path1.add(new Pose2d(6.3126, -3.6092, Rotation2d.fromDegrees(120)));

		trajectoryConfig.setReversed(true);
		trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

		ArrayList<Pose2d> path2 = new ArrayList<>();
		path2.add(new Pose2d(6.3126, -3.6092, Rotation2d.fromDegrees(120)));
		path2.add(new Pose2d(4.1312, -1.4694, Rotation2d.fromDegrees(137)));
		path2.add(new Pose2d(1.9804, 0.2412, Rotation2d.fromDegrees(149)));

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
		Pose2d initalPose = new Pose2d(3.5708, -2.9, Rotation2d.fromDegrees(180));

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
		while(!drive.isFinished()) {
			if(drive.getRamseteCompletePercent() > 0.1){
				intake.setDeployState(DeployState.UNDEPLOY);
			}
			if(isDead()) return;
		};
		System.out.println("here2");


		shootBallsTimed(20);

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