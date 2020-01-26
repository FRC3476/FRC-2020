package frc.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utility.Threaded;
import frc.subsystem.*;


public class CollisionManager extends Threaded { 
   
    Elevator elevator = Elevator.getInstance();
    
    boolean waitingOnElevator = false; 
    boolean waitingOnIntake = false;
    long prevTime;

   
    private static final CollisionManager cm = new CollisionManager();

   
    public static CollisionManager getInstance() {
        return cm;
    }    
    
    public CollisionManager() {
        setPeriod(Duration.ofMillis(20));
    }

    
    @Override
    synchronized public void update() {
        double starttime = Timer.getFPGATimestamp();

        
       
    }


}
