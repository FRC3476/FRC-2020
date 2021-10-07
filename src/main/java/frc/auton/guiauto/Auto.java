package frc.auton.guiauto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.auton.TemplateAuto;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.VisionManager.VisionStatus;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;


public class Auto {
    private static Auto instance = new Auto();

    public static Auto getInstance(){
        return instance;
    }

    static TemplateAuto context;

    public static void setContext(TemplateAuto autoContext){
        context = autoContext;
    }

    private Auto(){

    }

    Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager vision = VisionManager.getInstance();
	Shooter shooter = Shooter.getInstance();
	Hopper hopper = Hopper.getInstance();

    public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}


	public void turnOffIntakeTrack() {
		intake.setSpeed(0);
		hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
	}

	synchronized public void shootBalls (int amountOfBalls){
		Shooter shooter = Shooter.getInstance();
		VisionManager vision = VisionManager.getInstance();


		Translation2D target = new Translation2D(0, 67);
		Translation2D robot = context.here();

		Rotation2D pointAtTarget = robot.getAngle(target);
		System.out.println(target);
		drive.setRotation(pointAtTarget);
  
		//while(!drive.isFinished()) if(isDead()) return;
		System.out.println("finsihed drive");

		vision.setState(VisionStatus.AIMING);
		while (!shooter.isShooterSpeedOKAuto()) if(context.isDead()) return;
		System.out.println("shooter speed ok");
		vision.setState(VisionStatus.IDLE);
		System.out.println("trying to shoot");
		vision.setState(VisionStatus.WIN);
		while(!vision.isFinished()) if(context.isDead()) return;
		System.out.println("vision finished");
		//shooter.setFiring(true);

		double TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTimePerBall*3;
		
		while (Timer.getFPGATimestamp() < TargetTime) if(context.isDead()) return;

		shooter.setSpeed(0);

		//shooter.setFiring(false);
		vision.setState(VisionStatus.IDLE);

	}

}
