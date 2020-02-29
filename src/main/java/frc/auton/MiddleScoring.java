package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.Shooter;
import frc.subsystem.Shooter.ShooterState;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;

import frc.subsystem.Intake;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;

@SuppressWarnings("unused")
public class MiddleScoring implements Runnable { 
    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    Intake intake = Intake.getInstance();
    VisionManager vision = VisionManager.getInstance();

    int side = 1;

    private double TargetTime;

    boolean killSwitch = false;
    //Translation 2D is in inches.  

    public MiddleScoring(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
    }

    public MiddleScoring(Translation2D start, int side) {
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
    double ShooterFinishTime = 0;
    int stage = 0;
    @Override
    public void run() {

        //vision.setState();
        while(!vision.isFinished()) if(isDead()) return;
        //shooter.Shoot();

        TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTime;
        while (Timer.getFPGATimestamp() < TargetTime) if(isDead()) return;
        //shooter.StopShoot();

        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(75, -194), 60); 
        p1.addPoint(new Translation2D(20, -75), 60);
        p1.addPoint(new Translation2D(-25, -148), 60);
        p1.addPoint(new Translation2D(41, -80), 60);
        p1.addPoint(new Translation2D(80, -99), 60);


        drive.setAutoPath(p1, false);

        intake.setDeployState(DeployState.DEPLOY);
        intake.setIntakeState(IntakeState.INTAKE);

        while(!drive.isFinished()) if(isDead()) return;

        intake.setDeployState(DeployState.UNDEPLOY);
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(20, -75), 60); // Random Speed

        //vision.setState();

        while(!vision.isFinished()) if(isDead()) return;
        //shooter.Shoot();

        TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTime;

        while(Timer.getFPGATimestamp() < TargetTime) if(isDead()) return;
        //shooter.StopShoot();
    }
}
