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
public class TrenchDashRed extends TemplateAuto {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;

	Trajectory trajectory1Left;
	Trajectory trajectory2Left;

	Trajectory trajectory1Right;
	Trajectory trajectory2Right;


	public TrenchDashRed() {
		//Left Side
		{
			TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(100), Units.inchesToMeters(80));
			trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(25)));

			ArrayList<Pose2d> path1 = new ArrayList<>();
			path1.add(new Pose2d(3.6098, 3.3663, Rotation2d.fromDegrees(-180)));
			path1.add(new Pose2d(6.26, 3.3663, Rotation2d.fromDegrees(180)));
			path1.add(new Pose2d(9.21, 3.3663, Rotation2d.fromDegrees(180)));

			trajectoryConfig.setReversed(true);
			trajectory1Left = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

			ArrayList<Pose2d> path2 = new ArrayList<>();
			path2.add(new Pose2d(7.865, 3.3663, Rotation2d.fromDegrees(180)));
			path2.add(new Pose2d(3.7911, 3.3663, Rotation2d.fromDegrees(180)));
			
			trajectoryConfig.setReversed(false);
			trajectory2Left = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);
		}

		
		//Right Side
		{
			TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(100), Units.inchesToMeters(80));
			trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(25)));

			ArrayList<Pose2d> path1 = new ArrayList<>();
			path1.add(new Pose2d(3.6098, 3.3663, Rotation2d.fromDegrees(-180)));
			path1.add(new Pose2d(6.26, 3.3663, Rotation2d.fromDegrees(180)));
			path1.add(new Pose2d(9.21, 3.3663, Rotation2d.fromDegrees(180)));

			trajectoryConfig.setReversed(true);
			trajectory1Right = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

			ArrayList<Pose2d> path2 = new ArrayList<>();
			path2.add(new Pose2d(7.865, 3.3663, Rotation2d.fromDegrees(180)));
			path2.add(new Pose2d(3.7911, 3.3663, Rotation2d.fromDegrees(180)));
			
			trajectoryConfig.setReversed(false);
			trajectory2Right = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);
		}
	}

	@Override
	public void run() {

		Pose2d initalPose = new Pose2d(3.6098, 3.3663, Rotation2d.fromDegrees(180));
		System.out.println(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
		RobotTracker.getInstance().setInitialTranslation(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
		RobotTracker.getInstance().setInitialRotation(Rotation2D.fromWPIRotation2d(initalPose.getRotation()));
		
		Trajectory trajectory1;
		Trajectory trajectory2;
		if(side == 1){
			trajectory1 = trajectory1Left;
			trajectory2 = trajectory2Left;
		} else {
			trajectory1 = trajectory1Right;
			trajectory2 = trajectory2Right;
		}

		
		System.out.println("Trench Dash");
		shooter.setSpeed(4000);
		shooter.setHoodAngle(41);
		if(!shootBalls(3)) return;
		shooter.setHoodAngle(75);

		turnOnIntakeTrack();
		drive.setAutoPath(trajectory1);
		while(!drive.isFinished()) if(isDead()) return;
		intake.setDeployState(DeployState.UNDEPLOY);
		shooter.setSpeed(4000);

		System.out.println("here1");
		drive.setAutoPath(trajectory2);
		while(!drive.isFinished()) {
			if(drive.getRamseteCompletePercent() > 0.6){
				setupShooter();
				intake.setDeployState(DeployState.UNDEPLOY);
			} else if(drive.getRamseteCompletePercent() > .4){
				intake.setDeployState(DeployState.UNDEPLOY);
			} else if(drive.getRamseteCompletePercent() > .2){
				intake.setDeployState(DeployState.DEPLOY);
			}
			
			if(isDead()) return;
		}
		System.out.println("here2");
		if(!shootBalls(5)) return;

 
		synchronized (this) {
			done = true; 
		}


		
	}

}