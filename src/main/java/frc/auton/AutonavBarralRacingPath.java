package frc.auton;

import java.text.DecimalFormat;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.subsystem.RobotTracker;
import frc.utility.OrangeUtility;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class AutonavBarralRacingPath extends TemplateAuto {
    Trajectory trajectory;

    public AutonavBarralRacingPath() {
        super(new Translation2D(42,90));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
        
    }

    @Override
    public void run() {
        ArrayList<Pose2d> points = new ArrayList<>();
        points.add(new Pose2d(new Translation2D(42,90).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
		points.add(new Pose2d(new Translation2D(150,90).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
		points.add(new Pose2d(new Translation2D(190,60).getScaledWPITranslation2d(), Rotation2d.fromDegrees(-90)));
		points.add(new Pose2d(new Translation2D(150,20).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180)));
		points.add(new Pose2d(new Translation2D(110,60).getScaledWPITranslation2d(), Rotation2d.fromDegrees(90)));
		points.add(new Pose2d(new Translation2D(150,90).getScaledWPITranslation2d(), Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(new Translation2D(240,90).getScaledWPITranslation2d(), Rotation2d.fromDegrees(15)));
        points.add(new Pose2d(new Translation2D(240,120).getScaledWPITranslation2d(), Rotation2d.fromDegrees(90)));
        points.add(new Pose2d(new Translation2D(220,145).getScaledWPITranslation2d(), Rotation2d.fromDegrees(180)));
        points.add(new Pose2d(new Translation2D(180,100).getScaledWPITranslation2d(), Rotation2d.fromDegrees(-90)));
 
		TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(90), Units.inchesToMeters(200));
		config.setReversed(false);
		config.setEndVelocity(10);
        config.addConstraint(new DifferentialDriveKinematicsConstraint(drive.diffDriveKinematics, Units.inchesToMeters(90)));
        config.addConstraint(new CentripetalAccelerationConstraint(5));
		trajectory = TrajectoryGenerator.generateTrajectory(points, config);

		System.out.println("time " + trajectory.getTotalTimeSeconds());
		
		//OrangeUtility.sleep(10000);

		drive.setAutoPath(trajectory);
		while(!drive.isFinished()){
            DecimalFormat df = new DecimalFormat();
			System.out.println("robot angle: " + df.format(robotTracker.getOdometry().rotationMat.getDegrees()) + " position: X: " + 
			df.format(robotTracker.getOdometry().translationMat.getX()) + " Y: " + df.format(robotTracker.getOdometry().translationMat.getY()));
            if(isDead()) return;
            OrangeUtility.sleep(50);
        } 

        System.out.println("done");
        
        synchronized(this){
            done = true;
		}
    }

    
}
