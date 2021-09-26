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
public class TrenchDash extends TemplateAuto implements Runnable  {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;

	Trajectory trajectory1;
	Trajectory trajectory2;
	Trajectory trajectory3;
	Trajectory trajectory4;


	public TrenchDash() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(-124, /*-(165-55.5/2)*/  +(106.5-17)/*-131 */));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));

		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(50), Units.inchesToMeters(59.42928));
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(20)));

		ArrayList<Pose2d> path1 = new ArrayList<>();
		path1.add(new Pose2d(Units.inchesToMeters(-124), Units.inchesToMeters(+(106.5-17)), Rotation2d.fromDegrees(180+27)));
		path1.add(new Pose2d(Units.inchesToMeters(-124), Units.inchesToMeters(+(106.5-17)-120), Rotation2d.fromDegrees(180)));

		trajectoryConfig.setReversed(true);
		trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

		ArrayList<Pose2d> path2 = new ArrayList<>();
		path2.add(new Pose2d(Units.inchesToMeters(-124), Units.inchesToMeters(+(106.5-17)-120), Rotation2d.fromDegrees(180)));
		path2.add(new Pose2d(Units.inchesToMeters(-124+50), Units.inchesToMeters(+(106.5-17)), Rotation2d.fromDegrees(180)));
		
		trajectoryConfig.setReversed(false);
		trajectory2 = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);

		ArrayList<Pose2d> path3 = new ArrayList<>();
		path3.add(new Pose2d(Units.inchesToMeters((-124+50)), Units.inchesToMeters(+(106.5-17)), Rotation2d.fromDegrees(180)));
		path3.add(new Pose2d(Units.inchesToMeters(-24), Units.inchesToMeters(+124), Rotation2d.fromDegrees(180)));
		path3.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(22)).translateBy(frontBumpDirRight.scale(-32)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(150)));
		path3.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(22)).translateBy(frontBumpDirRight.scale(19)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(22)));
		path3.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(22)).translateBy(frontBumpDirRight.scale(19)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180-90+22)));
		path3.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(32)).translateBy(frontBumpDirRight.scale(19)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180+22)));
		path3.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(32)).translateBy(frontBumpDirRight.scale(5)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180+22+90)));
		path3.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(45)).translateBy(frontBumpDirRight.scale(5)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180+22)));
	
		
		trajectoryConfig.setReversed(true);
		trajectory3 = TrajectoryGenerator.generateTrajectory(path3, trajectoryConfig);

		ArrayList<Pose2d> path4 = new ArrayList<>();
		path4.add(new Pose2d(new Translation2D(-24, -124).translateBy(frontBumpDirLeft.scale(45)).translateBy(frontBumpDirRight.scale(5)).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180+22)));
		//path4.add(new Pose2d(Units.inchesToMeters(-124), Units.inchesToMeters(-(106.5-17)-124), Rotation2d.fromDegrees(180)));
		path4.add(new Pose2d(Units.inchesToMeters(-124+50), Units.inchesToMeters(-(106.5-17)), Rotation2d.fromDegrees(180)));
		
		trajectoryConfig.setReversed(false);
		trajectory4 = TrajectoryGenerator.generateTrajectory(path4, trajectoryConfig);


	}

	public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

	@Override
	public void run() {

		//Start 120 275
		System.out.println("TrenchRun");


		
		drive.setAutoPath(trajectory1);
		while(!drive.isFinished()) if(isDead()) return;

		System.out.println("here1");

		drive.setAutoPath(trajectory2);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println("here2");

		drive.setAutoPath(trajectory3);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println("here3");

		drive.setAutoPath(trajectory4);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println("here4");
		
 
		synchronized (this) {
			done = true; 
		}


		
	}

}