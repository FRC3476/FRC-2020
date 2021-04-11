package frc.auton;

import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class TurnTest extends TemplateAuto {

    public TurnTest() {
        super(new Translation2D(0, 0));
        robotTracker.setInitialRotation(Rotation2D.fromDegrees(0));
        // TODO Auto-generated constructor stub
    }


    @Override
    public void run() {
        drive.setRotationTeleop(Rotation2D.fromDegrees(90));

        while(!drive.isFinished()) if(isDead()) return;

        synchronized (this) {
            done = true; 
        }
    }


    
}
