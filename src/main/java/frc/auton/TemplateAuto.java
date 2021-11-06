package frc.auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.VisionManager.VisionStatus;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.visionlookup.ShooterPreset;
import frc.utility.visionlookup.VisionLookUpTable;

@SuppressWarnings("unused")
public abstract class TemplateAuto implements Runnable {
	protected Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();
	Hopper hopper = Hopper.getInstance();
	//12.68, 30.6
	Translation2D frontBumpDirRight = new Translation2D(12.68, -30.6).getUnitVector();
	Translation2D frontBumpDirLeft = new Translation2D(30.6, 12.68).getUnitVector();

	//CollisionManager collisionManager = CollisionManager.getInstance();
	
	int side = 1;

	boolean killSwitch = false;
	protected boolean done = false; 
	//Translation 2D is in inches.

	public TemplateAuto(Translation2D start) {
		RobotTracker.getInstance().setInitialTranslation(start);
	}

	public TemplateAuto() {
	}

	public TemplateAuto(Translation2D start, int side) {
		RobotTracker.getInstance().setInitialTranslation(start);
		this.side = side;
	}

	public Translation2D here() {
		return RobotTracker.getInstance().getOdometry().translationMat;
	}
	
	public Rotation2D dir() {
		return RobotTracker.getInstance().getOdometry().rotationMat;
	}

	synchronized public void killSwitch() {
		killSwitch = true;
	}

	synchronized public boolean isDead() {
		return killSwitch;
	}

	synchronized public boolean isFinished() {
		return done; 
	}

	@Override
	public abstract void run();
	
	public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setIntakeState(IntakeState.INTAKE);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}


	public void turnOffIntakeTrack() {
		intake.setIntakeState(IntakeState.OFF);
		hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
	}

	synchronized public boolean shootBalls(float amountOfBalls){
		return shootBallsTimed(3);
	}
	synchronized public boolean shootBallsTimed (float amountOfBalls){
		Shooter shooter = Shooter.getInstance();
		VisionManager vision = VisionManager.getInstance();
		setupShooter();

		Translation2D target = new Translation2D(0, 67);
		Translation2D robot = here();

		Rotation2D pointAtTarget = robot.getAngle(target);
		System.out.println(target);
		drive.setRotation(pointAtTarget);

		setupShooter();

		vision.setState(VisionStatus.AIMING);
		while (shooter.getTargetSpeed() < 100 || !shooter.isShooterSpeedOKAuto()) {
			if(isDead()) return false;
			System.out.println("shooter not ok: " + shooter.getRPM() + " target: " + shooter);
			setupShooter();

			OrangeUtility.sleep(50);
		}
		System.out.println("shooter speed ok");

		vision.setState(VisionStatus.IDLE);
		System.out.println("trying to shoot");
		vision.setState(VisionStatus.WIN);
		while(!vision.isFinished()){
			setupShooter();
			if(isDead()) return false;
		}
		System.out.println("vision finished");
		//shooter.setFiring(true);
		ShooterPreset sp = VisionLookUpTable.getInstance().getShooterPreset(Limelight.getInstance().getDistance());
		System.out.println("Shooting Auto Using: time: " + (15-DriverStation.getInstance().getMatchTime()) +  " distance: " + Limelight.getInstance().getDistance()+  "flywheel speed: " +sp.getFlywheelSpeed() + " wanted hood angle: " + sp.getHoodEjectAngle());

		double TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTimePerBall*amountOfBalls;
		
		while (Timer.getFPGATimestamp() < TargetTime) {
			if(isDead()) return false;
		}
		shooter.setSpeed(0);

		//shooter.setFiring(false);
		vision.setState(VisionStatus.IDLE);

		return true;

	}


	protected void setupShooter(){
		ShooterPreset sp = VisionLookUpTable.getInstance().getShooterPreset(Limelight.getInstance().getDistance());
		shooter.setSpeed(sp.getFlywheelSpeed());
		shooter.setHoodAngle(sp.getHoodEjectAngle());
	}

	public void reset() {
		this.killSwitch = false;
		this.done = false; 
	}

	public void setSide(int side) {
		this.side = side;
	}



}
