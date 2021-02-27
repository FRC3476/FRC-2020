package frc.auton;

import frc.utility.OrangeUtility;
import frc.utility.control.motion.Path;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class PowerPort extends TemplateAuto{
    public PowerPort() {
        super(new Translation2D(210, 90));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
    }

    @Override
    public void run() {
        while(!isDead()){
            Path p1 = new Path(here());
            p1.addPoint(new Translation2D(120, 90), 30);
            drive.setAutoPath(p1, false);
            while(! drive.isFinished()) if (isFinished()) return;
            shootBalls(3);

            Path p2 = new Path(here());
            p2.addPoint(new Translation2D(300, 90), 30);
            turnOnIntakeTrack();
            drive.setAutoPath(p2, true);
            while(! drive.isFinished()) if (isFinished()) return;
            OrangeUtility.sleep(3000);
            turnOffIntakeTrack();
        }
    }
}
