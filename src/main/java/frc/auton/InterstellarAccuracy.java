package frc.auton;

import frc.utility.OrangeUtility;
import frc.utility.control.motion.Path;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class InterstellarAccuracy extends TemplateAuto {

    public InterstellarAccuracy() {
        super(new Translation2D(90, 90));
        // TODO Auto-generated constructor stub
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
    }

    @Override
    public void run() {
        shootBalls(3);
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(330, 90), 30);
        turnOnIntakeTrack();
        drive.setAutoPath(p1, true);
        while(! drive.isFinished()) if (isFinished()) return;
        OrangeUtility.sleep(3000);
        turnOffIntakeTrack();

        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(150, 90), 30);
        drive.setAutoPath(p2, false);
        while(! drive.isFinished()) if (isFinished()) return;
        shootBalls(3);

        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(330, 90), 30);
        turnOnIntakeTrack();
        drive.setAutoPath(p3, true);
        while(! drive.isFinished()) if (isFinished()) return;
        OrangeUtility.sleep(3000);
        turnOffIntakeTrack();

        Path p4 = new Path(here());
        p4.addPoint(new Translation2D(210, 90), 30);
        drive.setAutoPath(p4, false);
        while(! drive.isFinished()) if (isFinished()) return;
        shootBalls(3);

        Path p5 = new Path(here());
        p5.addPoint(new Translation2D(330, 90), 30);
        turnOnIntakeTrack();
        drive.setAutoPath(p5, true);
        while(! drive.isFinished()) if (isFinished()) return;
        OrangeUtility.sleep(3000);
        turnOffIntakeTrack(); 

        Path p6 = new Path(here());
        p6.addPoint(new Translation2D(270, 90), 30);
        drive.setAutoPath(p6, false);
        while(! drive.isFinished()) if (isFinished()) return;
        shootBalls(3);

    }
    
}
