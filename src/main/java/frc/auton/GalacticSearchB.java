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

        Path path;
        if(Limelight.getInstance().isTargetVisiable()) {
            System.out.println("Blue Path");
            RobotTracker.getInstance().setInitialTranslation(new Translation2D(45, 60));
            path = new Path(here());
            //Blue Path Points
            path.addPoint(new Translation2D(180,60), 90);
            path.addPoint(new Translation2D(240,120), 90);
            path.addPoint(new Translation2D(275,60), 90);
            path.addPoint(new Translation2D(325,30), 90);
            isBlue = true;
            drive.setAutoPath(path, true);
        } else {
            System.out.println("Red Path");
            RobotTracker.getInstance().setInitialTranslation(new Translation2D(45, 120));
            path = new Path(here());
            //Red Path Points
            path.addPoint(new Translation2D(60,120), 80);
            //path.addPoint(new Translation2D(90,120), 80);
            path.addPoint(new Translation2D(90,120), 80);
            path.addPoint(new Translation2D(150,60), 80);
            path.addPoint(new Translation2D(210,120), 80);
            // path.addPoint(new Translation2D(300,60), 30);
            path.addPoint(new Translation2D(314,120), 80);
            isBlue = false;
            drive.setAutoPath(path, true);
        }
        
        
        // Turns On Intake
        turnOnIntakeTrack();

        // Checks Which Path To Run Depending On Boolean Change
        

        while(!drive.isFinished()) {
            if(isDead()) {
                return;
            }
            if(path.getPercentage() >= 0.95){
                intake.setDeployState(Intake.DeployState.UNDEPLOY);
                intake.setSpeed(0);
                hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
                hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
                return;
            }
        }

		synchronized (this) {
			done = true; 
		}
    }

}
