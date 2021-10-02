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
		//THIS WONT WORK
		super(new Translation2D(120, /*-(165-55.5/2)*/  -240/*-131 */));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));

		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(60), Units.inchesToMeters(30.42928));
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(20)));

		ArrayList<Pose2d> path1 = new ArrayList<>();
		path1.add(new Pose2d(3.6098, 3.3663, Rotation2d.fromDegrees(180)));
		path1.add(new Pose2d(6.9357, 3.3663, Rotation2d.fromDegrees(180)));

		trajectoryConfig.setReversed(true);
		trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

		ArrayList<Pose2d> path2 = new ArrayList<>();
		path2.add(new Pose2d(6.9357, 3.3663, Rotation2d.fromDegrees(180)));
		path2.add(new Pose2d(3.1465, 2.3712, Rotation2d.fromDegrees(-166)));
		
		trajectoryConfig.setReversed(false);
		trajectory2 = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);

		ArrayList<Pose2d> path3 = new ArrayList<>();
		path3.add(new Pose2d(3.1465, 2.3712, Rotation2d.fromDegrees(-166)));
		path3.add(new Pose2d(6.3608, 1.1321, Rotation2d.fromDegrees(113)));
		path3.add(new Pose2d(6.7916, 0.1015, Rotation2d.fromDegrees(112)));
		path3.add(new Pose2d(7.1633, 0.0508, Rotation2d.fromDegrees(-158)));
		path3.add(new Pose2d(7.4336, 0.3634, Rotation2d.fromDegrees(-76)));
		path3.add(new Pose2d(7.2309, 0.9885, Rotation2d.fromDegrees(-70)));
		path3.add(new Pose2d(7.611, 1.3179, Rotation2d.fromDegrees(-158)));
		path3.add(new Pose2d(5.6815, 1.9754, Rotation2d.fromDegrees(1)));
		path3.add(new Pose2d(3.2327, 1.8413, Rotation2d.fromDegrees(3)));
	
		
		trajectoryConfig.setReversed(true);
		trajectory3 = TrajectoryGenerator.generateTrajectory(path3, trajectoryConfig);


	}

	public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

	@Override
	public void run() {

		robotTracker.setInitialTranslation(new Translation2D(-124, /*-(165-55.5/2)*/  -(106.5-17)/*-131 */));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));

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
		
 
		synchronized (this) {
			done = true; 
		}


		
	}

}