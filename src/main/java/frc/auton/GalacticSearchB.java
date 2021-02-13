package frc.auton;

import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.math.*;
import frc.utility.Limelight;
import frc.utility.control.motion.Path;

import frc.subsystem.Intake;

public class GalacticSearchB extends TemplateAuto {
    Drive drive = Drive.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    Intake intake = Intake.getInstance();
    VisionManager vision = VisionManager.getInstance();
    Shooter shooter = Shooter.getInstance();

    boolean killSwitch = false;

    public GalacticSearchB() {
        super(new Translation2D(45, 60));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
    }

    public void turnOnIntakeTrack() {
		intake.setDeployState(Intake.DeployState.DEPLOY);
		intake.setSpeed(Constants.IntakeMotorPower);
		hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
		hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
	}

    public void run(){
        
        
        Boolean isBlue;
        
        
        
        

        // Vision Stuff To Decide Which Will Decide Which Path, And Set var isBlue accordingly (Is manual right now)

        if(Limelight.getInstance().isTargetVisiable()) {
            System.out.println("Blue Path");
            RobotTracker.getInstance().setInitialTranslation(new Translation2D(45, 60));
            Path bluePath = new Path(here());
            //Blue Path Points
            bluePath.addPoint(new Translation2D(180,60), 50);
            bluePath.addPoint(new Translation2D(240,120), 50);
            bluePath.addPoint(new Translation2D(300,60), 50);
            bluePath.addPoint(new Translation2D(314,30), 50);
            isBlue = true;
            drive.setAutoPath(bluePath, true);
        } else {
            System.out.println("Red Path");
            RobotTracker.getInstance().setInitialTranslation(new Translation2D(45, 120));
            Path redPath = new Path(here());
            //Red Path Points
            redPath.addPoint(new Translation2D(150,60), 50);
            redPath.addPoint(new Translation2D(210,120), 50);
            // redPath.addPoint(new Translation2D(300,60), 30);
            redPath.addPoint(new Translation2D(345,120), 50);
            isBlue = false;
            drive.setAutoPath(redPath, true);
        }
        
        // Turns On Intake
        turnOnIntakeTrack();

        // Checks Which Path To Run Depending On Boolean Change
        

        while(!drive.isFinished()) {
            if(isDead()) {
                return;
            }
            if(isBlue) {
                if(bluePath.getPercentage() >= 90){
                    intake.setDeployState(Intake.DeployState.UNDEPLOY);
                    intake.setSpeed(0);
                    hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
                    hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
                    return;
                }
            } else {
                if(redPath.getPercentage() >= 90){
                    intake.setDeployState(Intake.DeployState.UNDEPLOY);
                    intake.setSpeed(0);
                    hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
                    hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
                    return;
                }
            }
        }

		synchronized (this) {
			done = true; 
		}
    }
}