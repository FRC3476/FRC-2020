package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.VisionManager.VisionStatus;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;

@SuppressWarnings("unused")
public class TemplateAuto implements Runnable { 
    Drive drive = Drive.getInstance();
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
    boolean done = false; 
    //Translation 2D is in inches.

    public TemplateAuto(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
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
    public void run() {

    }
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
        Translation2D robot = here();

        Rotation2D pointAtTarget = robot.getAngle(target);
        System.out.println(target);
        drive.setRotation(pointAtTarget);
  
        //while(!drive.isFinished()) if(isDead()) return;
        //System.out.println("finsihed drive");

        vision.setState(VisionStatus.AIMING);
        while (!shooter.isShooterSpeedOKAuto()) if(isDead()) return;
        //System.out.println("shooter speed ok");
        vision.setState(VisionStatus.IDLE);

        vision.setState(VisionStatus.WIN);
        while(!vision.isFinished()) if(isDead()) return;
        //System.out.println("vision finished");
        //shooter.setFiring(true);

        double TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTimePerBall*3;

        while (Timer.getFPGATimestamp() < TargetTime) if(isDead()) return;

        shooter.setSpeed(0);

        //shooter.setFiring(false);
        vision.setState(VisionStatus.IDLE);

    }

}
