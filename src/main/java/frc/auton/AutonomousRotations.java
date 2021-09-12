package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import frc.utility.ShooterPreset;
import frc.utility.VisionLookUpTable;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;


@SuppressWarnings("unused")
public class AutonomousRotations extends TemplateAuto implements Runnable {

	double startY;

	private double TargetTime;

	boolean killSwitch = false;

	public AutonomousRotations() {
		//RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
		super(new Translation2D(0, 0));
		robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));

	}

	Limelight limelight = Limelight.getInstance();
	VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();

	public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

	@Override
	public void run() {
		System.out.println("AutonomousRotations");
		Path p1 = new Path(here());
		//TODO: Make Path 1 HERE 
		//Drive to the balls and pick them up



		turnOnIntakeTrack();
		drive.setAutoPath(p1, true);
		while(!drive.isFinished()) if(isDead()) return;
		turnOffIntakeTrack();

		Path p2 = new Path(here());
		//TODO: Make Path 1 HERE 
		//Drive back and shoot balls



		drive.setAutoPath(p2, false);
		while(!drive.isFinished()) if(isDead()) return;

		ShooterPreset sp = visionLookUpTable.getShooterPreset(limelight.getDistance());
		System.out.println("flywheel speed: " + sp.getFlyWheelSpeed() + " hood angle: " + sp.getHoodEjectAngle());
		shooter.setSpeed(sp.getFlyWheelSpeed());
		shooter.setHoodAngle(sp.getHoodEjectAngle());
		shootBalls(5);
		
 
		synchronized (this) {
			done = true; 
		}


		
	}

}