package frc.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
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
public class AutonavSlalomPath extends TemplateAuto implements Runnable  {
	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();

	private double TargetTime;
	
	boolean killSwitch = false;

    static Trajectory trajectory;

	public AutonavSlalomPath() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(42, 30));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
	}

	@Override
	public void run() {
        int speed = 75;

        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(60, 30), speed);
        p1.addPoint(new Translation2D(105, 90), speed);
        p1.addPoint(new Translation2D(250, 90), speed);
        p1.addPoint(new Translation2D(280, 10), speed);
        p1.addPoint(new Translation2D(320, 80), speed);
        p1.addPoint(new Translation2D(290, 70), speed);
        
        // drive.setAutoPath(p1, false);

        // while(!drive.isFinished()) if(isDead()) return;

        // Path p2 = new Path(here());

        p1.addPoint(new Translation2D(270, 40), speed);
        p1.addPoint(new Translation2D(250, 20), speed);
        p1.addPoint(new Translation2D(110, 25), speed);
        p1.addPoint(new Translation2D(80, 85), speed);
        p1.addPoint(new Translation2D(42, 90), speed);

        drive.setAutoPath(trajectory);

        while(!drive.isFinished()) if(isDead()) return;

		synchronized (this) {
			done = true;
		}
		
    }
    
    public static void calcTrajectories(){
        ArrayList<Pose2d> points = new ArrayList<>();
        points.add(new Pose2d(new Translation2D(42, 30).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(52, 30).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(105, 80).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(110, 80).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(220, 80).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(230, 80).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(275, 30).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(300, 30).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(320, 60).getScaledWPITranslation2d(), Rotation2d.fromDegrees(90)));
        points.add(new Pose2d(new Translation2D(305, 85).getScaledWPITranslation2d(), Rotation2d.fromDegrees(-160)));
        points.add(new Pose2d(new Translation2D(250, 40).getScaledWPITranslation2d(), Rotation2d.fromDegrees(-170)));
        points.add(new Pose2d(new Translation2D(130, 41).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180)));
        points.add(new Pose2d(new Translation2D(120, 45).getScaledWPITranslation2d(), Rotation2d.fromDegrees(170)));
        points.add(new Pose2d(new Translation2D(83, 83).getScaledWPITranslation2d(), Rotation2d.fromDegrees(170)));
        points.add(new Pose2d(new Translation2D(49, 88).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180)));

        TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(80), Units.inchesToMeters(100));
        config.setReversed(false);
        config.setEndVelocity(5);
        config.addConstraint(
                new DifferentialDriveKinematicsConstraint(Constants.RamseteDiffDriveKinematics, Units.inchesToMeters(80)));
        config.addConstraint(new CentripetalAccelerationConstraint(6));
        trajectory = TrajectoryGenerator.generateTrajectory(points, config);
        System.out.println("finished cacluclating slalom trajec");
    }

}
