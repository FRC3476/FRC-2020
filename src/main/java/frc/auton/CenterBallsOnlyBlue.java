package frc.auton;

import java.io.File;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.auton.guiauto.AbstractGuiAuto;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.visionlookup.ShooterPreset;
import frc.utility.visionlookup.VisionLookUpTable;
import frc.utility.control.motion.Path;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;

@SuppressWarnings("unused")
public class CenterBallsOnlyBlue extends AbstractGuiAuto {

	public CenterBallsOnlyBlue() {
		super(new File(Filesystem.getDeployDirectory().getPath() + "/auto/centerOnly.json")); //Load the json file 
	}

	@Override
	public void run() {
		drive.configRamsete(2.5, 0.7);
		super.run();
	}
	
	// boolean killSwitch = false;

	// Trajectory trajectory1;
	// Trajectory trajectory2;
	// Trajectory trajectory3;
	// Trajectory trajectory4;


	// public CenterBallsOnly() {
	// 	TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Units.inchesToMeters(60), Units.inchesToMeters(120.42928));
	// 	trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(Units.inchesToMeters(70)));

	// 	ArrayList<Pose2d> path1 = new ArrayList<>();
	// 	path1.add(new Pose2d(3.6084, 1.5921, Rotation2d.fromDegrees(180)));
	// 	path1.add(new Pose2d(6.6663, 1.7977, Rotation2d.fromDegrees(-179)));
	// 	path1.add(new Pose2d(7.6392, 1.4967, Rotation2d.fromDegrees(111)));
	// 	path1.add(new Pose2d(6.6037, 0.3038, Rotation2d.fromDegrees(23)));
	
		
	// 	trajectoryConfig.setReversed(true);
	// 	trajectory1 = TrajectoryGenerator.generateTrajectory(path1, trajectoryConfig);

	// 	ArrayList<Pose2d> path2 = new ArrayList<>();
	// 	path2.add(new Pose2d(6.6037, 0.3038, Rotation2d.fromDegrees(23)));
	// 	path2.add(new Pose2d(7.9773, 1.4059, Rotation2d.fromDegrees(76)));
		
	// 	trajectoryConfig.setReversed(false);
	// 	trajectory2 = TrajectoryGenerator.generateTrajectory(path2, trajectoryConfig);

	// 	ArrayList<Pose2d> path3 = new ArrayList<>();
	// 	path3.add(new Pose2d(7.9773, 1.4059, Rotation2d.fromDegrees(76)));
	// 	path3.add(new Pose2d(6.7618, -0.0346, Rotation2d.fromDegrees(23)));
		
	// 	trajectoryConfig.setReversed(true);
	// 	trajectory3 = TrajectoryGenerator.generateTrajectory(path3, trajectoryConfig);

	// 	ArrayList<Pose2d> path4 = new ArrayList<>();
	// 	path4.add(new Pose2d(6.7618, -0.0346, Rotation2d.fromDegrees(23)));
	// 	//path4.add(new Pose2d(6.6293, 1.4804, Rotation2d.fromDegrees(164)));
	// 	path4.add(new Pose2d(4.5171, 2.1728, Rotation2d.fromDegrees(-177)));
		
	// 	trajectoryConfig.setReversed(false);
	// 	trajectory4 = TrajectoryGenerator.generateTrajectory(path4, trajectoryConfig);
	
		
		


	// }

	// @Override
	// public void run() {
		
	// 	Pose2d initalPose = new Pose2d(3.6084, 1.5921, Rotation2d.fromDegrees(180));
	// 	robotTracker.setInitialTranslation(Translation2D.fromWPITranslation2d(initalPose.getTranslation()));
	// 	robotTracker.setInitialRotation(Rotation2D.fromWPIRotation2d(initalPose.getRotation()));

	// 	//Start 120 275
	// 	System.out.println("Center Only");

	// 	ShooterPreset sp1 = VisionLookUpTable.getInstance().getShooterPreset(Limelight.getInstance().getDistance());
	// 	//System.out.println("flywheel speed: " +sp.getFlyWheelSpeed() + " hood angle: " + sp.getHoodEjectAngle());
	// 	shooter.setSpeed(sp1.getFlyWheelSpeed());
	// 	shooter.setHoodAngle(sp1.getHoodEjectAngle());

	// 	shootBalls(3);
	// 	turnOnIntakeTrack();
	// 	drive.setAutoPath(trajectory1);
	// 	while(!drive.isFinished()) if(isDead()) return;
	// 	System.out.println("here1");

	// 	drive.setAutoPath(trajectory2);
	// 	while(!drive.isFinished()) if(isDead()) return;
	// 	System.out.println("here2");

	// 	drive.setAutoPath(trajectory3);
	// 	while(!drive.isFinished()) if(isDead()) return;
	// 	System.out.println("here3");

	// 	turnOffIntakeTrack();
	// 	intake.setDeployState(DeployState.UNDEPLOY);
	// 	shooter.setSpeed(4000);

	
	// 	drive.setAutoPath(trajectory4);
	// 	while(!drive.isFinished()){
	// 		if(isDead()) return;
	// 		if(Limelight.getInstance().isTargetVisiable() && Limelight.getInstance().getTagetArea()>= Constants.ShooterVisionMinimumTargetArea ){
	// 			ShooterPreset sp = VisionLookUpTable.getInstance().getShooterPreset(Limelight.getInstance().getDistance());
	// 			//System.out.println("flywheel speed: " +sp.getFlyWheelSpeed() + " hood angle: " + sp.getHoodEjectAngle());
	// 			shooter.setSpeed(sp.getFlyWheelSpeed());
	// 			shooter.setHoodAngle(sp.getHoodEjectAngle());
	// 		}
	// 		OrangeUtility.sleep(20);
	// 	} 
	// 	System.out.println("here4");

	// 	shootBallsTimed(10);
 
	// 	synchronized (this) {
	// 		done = true; 
	// 	}


		
	// }

}