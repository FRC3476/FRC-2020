package frc.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
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
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
	}

	@Override
	public void run() {
		
        Pose2d startPoint = robotTracker.getOdometryMeters();

        ArrayList<Translation2d> points = new ArrayList<>();

		points.add(new Translation2D(105,90).getScaledWPITranslation2d());
		points.add(new Translation2D(120,40).getScaledWPITranslation2d());
		points.add(new Translation2D(140,40).getScaledWPITranslation2d());
		points.add(new Translation2D(150,90).getScaledWPITranslation2d());
		Pose2d endPoint = new Pose2d(new Translation2D(170,150).getScaledWPITranslation2d(), Rotation2d.fromDegrees(-90));



		TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(Constants.MaxPathSpeed), Units.inchesToMeters(5));
		config.setReversed(true);
		config.addConstraint(new DifferentialDriveKinematicsConstraint(drive.diffDriveKinematics, Units.inchesToMeters(Constants.MaxPathSpeed)));
		//TODO add CentripetalAccelerationConstraint

		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPoint, points, endPoint, config);






		double time = Timer.getFPGATimestamp();
        Translation2D point1 = new Translation2D(70, 90);
        Translation2D point2 = new Translation2D(90, 150);
        
		Translation2D robot = here();

        Path p1 = new Path(here());
        p1.addPoint(point1, 60);
        p1.addPoint(point2, 60);
		drive.setAutoPath(p1, false);
		
		while(!drive.isFinished()) if(isDead()) return;
		
		Path p2 = new Path(here());
		p2.addPoint(new Translation2D(105,90), 80);
		p2.addPoint(new Translation2D(120,40), 70);
		p2.addPoint(new Translation2D(140,40), 72);
		p2.addPoint(new Translation2D(150,90), 75);
		p2.addPoint(new Translation2D(170,150), 80);
		drive.setAutoPath(p2, true);
		while(!drive.isFinished()) if(isDead()) return;
		
		Path p3 = new Path(here());
		p3.addPoint(new Translation2D(190,40), 70);
		p3.addPoint(new Translation2D(250,40), 70);
		p3.addPoint(new Translation2D(260,70), 70);
		p3.addPoint(new Translation2D(270,150), 70);
		drive.setAutoPath(p3, false);
		while(!drive.isFinished()) if(isDead()) return;

		Path p4 = new Path(here());
		p4.addPoint(new Translation2D(270,130), 70);
		p4.addPoint(new Translation2D(330,105), 70);
		drive.setAutoPath(p4, true);
		while(!drive.isFinished()) if(isDead()) return;
		System.out.println((time-Timer.getFPGATimestamp())*1000000);

        synchronized(this){
            done = true;
		}
		
	}

}
