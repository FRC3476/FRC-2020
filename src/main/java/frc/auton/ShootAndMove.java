package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
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
    double startY;

    private double TargetTime;
    
    boolean killSwitch = false;


    public ShootAndMove(double d) {
        //RobotTracker.getInstance().setInitialTranslation(new Translation2D(startX, 75));
        super(new Translation2D(120.5, d));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(180));
        this.startY = d;

    }

    @Override
    public void run() {

        //Start 75 120
        System.out.println("ShootAndMove");
        double turnAngle = 0;
        
        //turnAngle = Math.toDegrees(Math.atan2(75 , here().getX()-48));
        Translation2D target = new Translation2D(0, 67);
        Translation2D robot = here();

        Rotation2D pointAtTarget = robot.getAngle(target);
        System.out.println(target);



        drive.setRotation(pointAtTarget);
        
        while (!shooter.isHomed()) if(isDead()) return;
        shooter.setHoodAngle(33);
        shooter.setSpeed(5700);
        //while(!drive.isFinished()) if(isDead()) return;
        //System.out.println("finsihed drive");

        vision.setState(VisionStatus.AIMING);
        while (!shooter.isShooterSpeedOKAuto()) if(isDead()) return;
        System.out.println("shooter speed ok");
        vision.setState(VisionStatus.IDLE);

        vision.setState(VisionStatus.WIN);
        while(!vision.isFinished()) if(isDead()) return;
        System.out.println("vision finished");
        //shooter.setFiring(true);

        TargetTime = Timer.getFPGATimestamp() +Constants.AutoShooterOnTimePerBall*5;

        while (Timer.getFPGATimestamp() < TargetTime) if(isDead()) return;

        shooter.setSpeed(0);

        //shooter.setFiring(false);
        vision.setState(VisionStatus.IDLE);


        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(120.5+45, startY), 40);

        drive.setAutoPath(p1, true);
        while(!drive.isFinished()) if(isDead()) return;

        synchronized (this) {
            done = true; 
        }


        
    }

}
