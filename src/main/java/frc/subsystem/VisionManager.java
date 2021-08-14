package frc.subsystem;

import frc.robot.Constants;
import frc.utility.Limelight;
import frc.utility.math.Rotation2D;

public class VisionManager extends Subsystem { 
   

	long prevTime;
	VisionStatus visionStatus = VisionStatus.IDLE;
	Hopper hopper = Hopper.getInstance();
	Drive drive = Drive.getInstance();
	Shooter shooter = Shooter.getInstance();
	Limelight limelight = Limelight.getInstance();
	boolean shoot = false;
	BlinkinLED led;
	public int winStage = 0;
   
	private static final VisionManager vm = new VisionManager();


	public synchronized VisionStatus getState() {
		return visionStatus;
	}
   
	public static VisionManager getInstance() {
		return vm;
	}    
	
	private VisionManager() {
		super(Constants.VisionManagerPeriod);
		led = BlinkinLED.getInstance();
	}

	public enum VisionStatus {
		AIMING, IDLE, WIN
	}

	public void resetWin() {
		winStage = 0;
	}
	
	@Override
	synchronized public void update() {
		switch(visionStatus){
			case AIMING:
				
			if (limelight.isTargetVisiable()){            
				drive.setRotationTeleop(Rotation2D.fromDegrees((-RobotTracker.getInstance().getOdometry().rotationMat.getDegrees())+limelight.getHorizontalOffset()+Constants.cameraXOffset));
				
			}
				shoot = shoot || !drive.isAiming();

				//System.out.println("current angle: " + (-RobotTracker.getInstance().getOdometry().rotationMat.getDegrees()) + " wanted angle: " + (limelight.getHorizontalOffset()) + " shoot: " + shoot);
				
				if (shoot){
					led.setColor(.77);
				} else {
					led.setColor(-.11);
				}
				
				break;

			case IDLE:
				break;
			
			case WIN:
				if (limelight.isTargetVisiable()){            
					drive.setRotationTeleop(Rotation2D.fromDegrees((-RobotTracker.getInstance().getOdometry().rotationMat.getDegrees())+limelight.getHorizontalOffset()+Constants.cameraXOffset));
					
				}

				shoot = shoot || !drive.isAiming();
				//System.out.println("current angle: " + (-RobotTracker.getInstance().getOdometry().rotationMat.getDegrees()) + " wanted angle: " + (limelight.getHorizontalOffset()) + " shoot: " + shoot);

				if(shoot) {

					shooter.setFiring(true);
					hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
					hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
				 //   led.setColor(.77);

				} else {
					shooter.setFiring(false);
					hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
					hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
				   // led.setColor(-.11);

				   //System.out.println("ready: " + shoot);

				} 
				break;

		}
		
	   
	}

	public void stop() {
		shooter.setFiring(false);
		hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
		hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
		led.setColor(-.11);
		setState(VisionStatus.IDLE);
	}

	public synchronized void setState(VisionStatus state){
		if(state == VisionStatus.IDLE && visionStatus != VisionStatus.IDLE)
		{
			//Turn everything off the first time we go into idle
			drive.driveState = Drive.DriveState.DONE;
			shoot = false;
			shooter.setFiring(false);
			hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
			hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);

		}

		visionStatus = state;


	}

	public synchronized boolean isFinished(){
		return shoot; 
	}

	@Override
	public void selfTest() {

	}

	@Override
	public void logData() {

	}

}
