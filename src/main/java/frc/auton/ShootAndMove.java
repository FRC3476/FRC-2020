package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;

import frc.subsystem.Intake;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;


@SuppressWarnings("unused")
public class ShootAndMove extends TemplateAuto implements Runnable  {
    Drive drive = Drive.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    Intake intake = Intake.getInstance();
    VisionManager vision = VisionManager.getInstance();
    Shooter shooter = Shooter.getInstance();
    int startX;

    private double TargetTime;
    
    boolean killSwitch = false;


    public ShootAndMove(int startX) {
        //RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
        super(new Translation2D(startX, 75));
        this.startX = startX;

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

    @Override
    public void run() {

        //Start 75 120
        System.out.println("ShootAndMove");
        double turnAngle = 0;
        try{
            if (here().getX() != 48){
                turnAngle = Math.toDegrees(Math.atan2(75 , here().getX()-48));
            }
            
        } catch(Exception exception){
            exception.printStackTrace();
        }
        
        drive.setRotation(Rotation2D.fromDegrees(turnAngle));
        vision.setState(VisionStatus.AIMING);
        while(!vision.isFinished()) if(isDead()) return;

        shooter.setFiring(true);
        TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTime;
        while (Timer.getFPGATimestamp() < TargetTime) if(isDead()) return;

        shooter.setFiring(false);
        vision.setState(VisionStatus.IDLE);

        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(75, startX), 20);

        drive.setAutoPath(p1, false);
        while(!drive.isFinished()) if(isDead()) return;


        
    }

}
