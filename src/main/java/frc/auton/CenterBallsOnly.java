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
public class CenterBallsOnly extends TemplateAuto implements Runnable  {
   
	double startY;

	private double TargetTime;
	
	boolean killSwitch = false;

	Trajectory trajectory1;


	public CenterBallsOnly() {
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(60), Units.inchesToMeters(30.42928));
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(20)));

		ArrayList<Pose2d> path1 = new ArrayList<>();
		path1.add(new Pose2d(3.5975, 2.3712, Rotation2d.fromDegrees(180)));
		path1.add(new Pose2d(5.6506, 1.6797, Rotation2d.fromDegrees(180)));
		path1.add(new Pose2d(6.3608, 1.1321, Rotation2d.fromDegrees(113)));
		path1.add(new Pose2d(6.3608, 1.1321, Rotation2d.fromDegrees(113)));
		path1.add(new Pose2d(6.7916, 0.1015, Rotation2d.fromDegrees(112)));
		path1.add(new Pose2d(7.1633, 0.0508, Rotation2d.fromDegrees(-158)));
		path1.add(new Pose2d(7.4336, 0.3634, Rotation2d.fromDegrees(-76)));
		path1.add(new Pose2d(7.2309, 0.9885, Rotation2d.fromDegrees(-70)));
		path1.add(new Pose2d(7.611, 1.3179, Rotation2d.fromDegrees(-158)));
		path1.add(new Pose2d(5.6815, 1.9754, Rotation2d.fromDegrees(1)));
		path1.add(new Pose2d(3.2327, 1.8413, Rotation2d.fromDegrees(3)));
	
		
		trajectoryConfig.setReversed(true);
		trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);


	}

	public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

	@Override
	public void run() {
		
		Pose2d initalPose = new Pose2d(3.5975, 2.3712, Rotation2d.fromDegrees(180));
		robotTracker.setInitialTranslation(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
		robotTracker.setInitialRotation(Rotation2D.fromWPIRotation2d(initalPose.getRotation()));

		//Start 120 275
		System.out.println("Center Only");


		
		drive.setAutoPath(trajectory1);
		while(!drive.isFinished()) if(isDead()) return;

		System.out.println("here1");
 
		synchronized (this) {
			done = true; 
		}


		
	}

}