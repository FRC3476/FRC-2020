package frc.auton;

import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class SimpleAutoTemplate extends SimpleAutonomous {

    public SimpleAutoTemplate() {
        super(new Translation2D(), Rotation2D.fromDegrees(0), 40);
    }

    @Override
    public void run() {
        /*
        This is where you should start coding 
        For reference here are the methods that you can use. You should replace the stuff in brackets with a value of your own.
            1. moveForward([distanceInInches]);
            2. moveBackward([distanceInInches]);
            3. turn([angleInDegrees]); Note a negative angle means turn left and a positive angle means turn right.
        */





        //This needs to be the last line of code. Don't remove it or put anything below it.
        //This tells the robot the autonomous is done and that the robot can stop
        synchronized (this) {done = true;}
    }
    
}
