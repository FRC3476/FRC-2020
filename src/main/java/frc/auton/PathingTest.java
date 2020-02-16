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



public class PathingTest extends TemplateAuto implements Runnable {
    public PathingTest(int side, double startX) { 
        //Start position
        super(new Translation2D(0, 0), 0);
        
    }

    @Override
    public void run() {

        //Start 75 120
        System.out.println("Pathing test"); 
        
        
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(24, 12), 10); 
        p1.addPoint(new Translation2D(36,-12), 10);


        drive.setAutoPath(p1, false);

        while(!drive.isFinished()){
            System.out.println(drive.getGyroAngle());
            if(isDead()) return;
        } 
        
    }

}