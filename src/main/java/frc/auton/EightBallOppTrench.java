package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.Shooter.ShooterState;
import frc.subsystem.VisionManager.VisionStatus;


@SuppressWarnings("unused")
public class EightBallOppTrench extends TemplateAuto implements Runnable  {
   
    double startY;

    private double TargetTime;
    
    boolean killSwitch = false;


    public EightBallOppTrench(double d) {
        //RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
        super(new Translation2D(120.5, /*-(165-55.5/2)*/ -131));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));

    }

    public void turnOnIntakeTrack() {
        intake.setDeployState(Intake.DeployState.DEPLOY);
        intake.setSpeed(Constants.IntakeMotorPower);
        hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
    }

    @Override
    public void run() {

        //Start 120 275
        System.out.println("ShootAndMove");


        
        //Get oponent trench balls
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(243 - 35.0/2+6, -131), 130);
        intake.setDeployState(Intake.DeployState.DEPLOY);
        intake.setSpeed(Constants.IntakeMotorPower);
        hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
        drive.setAutoPath(p1, true);
        while(!drive.isFinished()) if(isDead()) return;
        shooter.setSpeed(5300);

        //Drive to shooting position abd fire 
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(125.5, 0), 130);
        p2.addPoint(new Translation2D(120.5, 0), 130);
        drive.setAutoPath(p2, false);
        while(!drive.isFinished()) if(isDead()) return;
        intake.setSpeed(0);
        hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
        hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
        shooter.setHoodAngle(33);
        shootBalls(5);
        shooter.setHoodAngle(0);

        //Grab first balls from switch bump
        //94+1205, 12
        Path p3 = new Path(here());
        p3.addPoint(new Translation2D( /*216*/ 214.5, /*-15*/ -12).translateBy(frontBumpDirRight.scale(/*-28.0*/ 0)).translateBy(frontBumpDirLeft.scale(/*-20.0*/ -50.0)), 120);
        p3.addPoint(new Translation2D(214.5, -12).translateBy(frontBumpDirRight.scale(0.0)).translateBy(frontBumpDirLeft.scale(-13.0)), 40);
        drive.setAutoPath(p3, true);
        turnOnIntakeTrack();
        while(!drive.isFinished()) if(isDead()) return;
        
        //Drive forward perp to bump 
        Path p4 = new Path(here());
        p4.addPoint(new Translation2D( /*216*/ 214.5, /*-15*/ -12).translateBy(frontBumpDirRight.scale(/*-28.0*/ -15.0)).translateBy(frontBumpDirLeft.scale(/*-20.0*/ -50.0)), 120);
        //p4.addPoint();
        drive.setAutoPath(p4, false);
        while(!drive.isFinished()) if(isDead()) return;

        //Grab second balls from switch bump
        Path p5 = new Path(here());
        p5.addPoint(new Translation2D(214.5, -12).translateBy(frontBumpDirRight.scale(-15.0)).translateBy(frontBumpDirLeft.scale(-13.0)), 40);
        drive.setAutoPath(p5, true);
        while(!drive.isFinished()) if(isDead()) return;

        //Drive back to shootng position
        Path p6 = new Path(here());
        p6.addPoint(new Translation2D(125.5, 0/*67*/), 130);
        p6.addPoint(new Translation2D(120.5, 0/*67*/), 130);
        drive.setAutoPath(p6, false);
        while(!drive.isFinished()) if(isDead()) return;

        shooter.setHoodAngle(33);
        shootBalls(5);
        shooter.setHoodAngle(0);
        
 
        synchronized (this) {
            done = true; 
        }


        
    }

}