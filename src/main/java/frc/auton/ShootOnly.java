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


@SuppressWarnings("unused")
public class ShootOnly implements Runnable {
    Drive drive = Drive.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    Intake intake = Intake.getInstance();
    VisionManager vision = VisionManager.getInstance();
    Shooter shooter = Shooter.getInstance();

    private double TargetTime;
    
    boolean killSwitch = false;




    public ShootOnly(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
    }

    public ShootOnly(Translation2D start, int side) {
        RobotTracker.getInstance().setInitialTranslation(start);
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
        System.out.println("Only Shoot"); 
        
        vision.aim();
        while(!vision.isFinished()) if(isDead()) return;
        //shooter.Shoot();

        TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTime;

        while (Timer.getFPGATimestamp() < TargetTime) if(isDead()) return;

        //shooter.StopShoot();

        
    }

}
