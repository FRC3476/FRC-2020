package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;


@SuppressWarnings("unused")
public class TenBall extends TemplateAuto implements Runnable  {
    Drive drive = Drive.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    Intake intake = Intake.getInstance();
    VisionManager vision = VisionManager.getInstance();
    Shooter shooter = Shooter.getInstance();
    Hopper hopper = Hopper.getInstance();
    double startY;

    private double TargetTime;
    
    boolean killSwitch = false;


    public TenBall(double d) {
        //RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
        super(new Translation2D(120.5, d));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
        this.startY = d;

    }

    @Override
    public void run() {

        //Start 120 275
        System.out.println("ShootAndMove");

        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(200, 275), 50);
        p1.addPoint(new Translation2D(243, 275), 50);

        intake.setDeployState(Intake.DeployState.DEPLOY);
        intake.setSpeed(Constants.IntakeMotorPower);
        hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);

        drive.setAutoPath(p1, true);
        while(!drive.isFinished()) if(isDead()) return;

        intake.setSpeed(0);
        hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);

        shooter.setHoodAngle(33);
        shooter.setSpeed(5300);

        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(200, 225), 50);
        p2.addPoint(new Translation2D(155, 150), 50);
        drive.setAutoPath(p2, false);
        while(!drive.isFinished()) if(isDead()) return;

        shootBalls(5);

        drive.setRotation(Rotation2D.fromDegrees(70)); //TODO: Probably needs changing
        while(!drive.isFinished()) if(isDead()) return;

        intake.setDeployState(Intake.DeployState.DEPLOY);
        intake.setSpeed(Constants.IntakeMotorPower);
        hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(180, 25), 50);
        p3.addPoint(new Translation2D(302, 25), 50);
        drive.setAutoPath(p3, true);
        while(!drive.isFinished()) if(isDead()) return;

        intake.setSpeed(0);
        hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);

        Path p4 = new Path(here());

        p4.addPoint(new Translation2D(214, 65),50);
        drive.setAutoPath(p4, false);
        while(!drive.isFinished()) if(isDead()) return;

        drive.setRotation(Rotation2D.fromDegrees(30)); //TODO: Probably needs changing
        while(!drive.isFinished()) if(isDead()) return;

        intake.setDeployState(Intake.DeployState.DEPLOY);
        intake.setSpeed(Constants.IntakeMotorPower);
        hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE , false);
        Path p5 = new Path(here());
        p5.addPoint(new Translation2D(239, 100), 50);
        drive.setAutoPath(p5, true);
        while(!drive.isFinished()) if(isDead()) return;

        Path p6 = new Path(here());
        p6.addPoint(new Translation2D(190, 75), 50);
        p6.addPoint(new Translation2D(150, 75), 50);

        drive.setAutoPath(p6, false);
        while(!drive.isFinished()) if(isDead()) return;

        shootBalls(5);

        synchronized (this) {
            done = true; 
        }


        
    }

}
