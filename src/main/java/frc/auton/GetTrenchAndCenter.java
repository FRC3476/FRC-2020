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
public class GetTrenchAndCenter extends TemplateAuto {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;

	Trajectory trajectory1;
	Trajectory trajectory2;
	Trajectory trajectory3;
	Trajectory trajectory4;
	Trajectory trajectory5;


	public GetTrenchAndCenter() {
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(120), Units.inchesToMeters(150));
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(50)));

		ArrayList<Pose2d> path1 = new ArrayList<>();
		path1.add(new Pose2d(3.6098, 3.3663, Rotation2d.fromDegrees(180)));
		path1.add(new Pose2d(6.7956, 3.3663, Rotation2d.fromDegrees(180)));

		trajectoryConfig.setReversed(true);
		trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

		ArrayList<Pose2d> path2 = new ArrayList<>();
		path2.add(new Pose2d(6.7956, 3.3663, Rotation2d.fromDegrees(180)));
		path2.add(new Pose2d(3.7911, 2.5525, Rotation2d.fromDegrees(-160)));
		
		trajectoryConfig.setReversed(false);
		trajectory2 = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);

		ArrayList<Pose2d> path3 = new ArrayList<>();
		path3.add(new Pose2d(3.7911, 2.5525, Rotation2d.fromDegrees(-160)));
		path3.add(new Pose2d(7.1891, 1.1162, Rotation2d.fromDegrees(113)));
		//path3.add(new Pose2d(7.2309, 0.9885, Rotation2d.fromDegrees(-70)));
		
		trajectoryConfig.setReversed(true);
		trajectory3 = TrajectoryGenerator.generateTrajectory(path3, trajectoryConfig);

		ArrayList<Pose2d> path4 = new ArrayList<>();
		path4.add(new Pose2d(7.1891, 1.1162, Rotation2d.fromDegrees(113)));
		path4.add(new Pose2d(6.2335, 1.7569, Rotation2d.fromDegrees(113)));
		
		trajectoryConfig.setReversed(false);
		trajectory4 = TrajectoryGenerator.generateTrajectory(path4, trajectoryConfig);

		ArrayList<Pose2d> path5 = new ArrayList<>();
		path5.add(new Pose2d(6.2335, 1.7569, Rotation2d.fromDegrees(113)));
		path5.add(new Pose2d(3.6841, 1.8062, Rotation2d.fromDegrees(-113)));
		
		trajectoryConfig.setReversed(false);
		trajectory5 = TrajectoryGenerator.generateTrajectory(path5, trajectoryConfig);



	}

	@Override
	public void run() {

		Pose2d initalPose = new Pose2d(3.6098, 3.3663, Rotation2d.fromDegrees(180));
		System.out.println(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
		RobotTracker.getInstance().setInitialTranslation(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
		RobotTracker.getInstance().setInitialRotation(Rotation2D.fromWPIRotation2d(initalPose.getRotation()));

		System.out.println("Trench Dash");
		turnOnIntakeTrack();
		drive.setAutoPath(trajectory1);
		while(!drive.isFinished()) if(isDead()) return;
		turnOffIntakeTrack();
		shooter.setSpeed(4000);
		shooter.setHoodAngle(41);

		System.out.println("here1");

		drive.setAutoPath(trajectory2);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println("here2");
		if(!shootBalls(5)) return;

		turnOnIntakeTrack();
		drive.setAutoPath(trajectory3);
		while(!drive.isFinished()) {
			if(isDead()) return;

			// if(drive.getRamseteCompletePercent()>0.9){
			// 	turnOffIntakeTrack();
			// 	shooter.setSpeed(4000);
			// 	shooter.setHoodAngle(41);
			// }
		}


		drive.setAutoPath(trajectory4);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println("here4");
		if(!shootBalls(5)) return; 
		
 
		synchronized (this) {
			done = true; 
		}


		
	}

}