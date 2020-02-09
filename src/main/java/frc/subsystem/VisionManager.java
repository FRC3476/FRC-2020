package frc.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utility.Threaded;
import frc.subsystem.*;


public class VisionManager extends Threaded { 
   
    Elevator elevator = Elevator.getInstance();
    
    boolean waitingOnElevator = false; 
    boolean waitingOnIntake = false;
    long prevTime;

   
    private static final VisionManager cm = new VisionManager();

   
    public static VisionManager getInstance() {
        return cm;
    }    
    
    public VisionManager() {
        setPeriod(Duration.ofMillis(20));
    }

    
    @Override
    synchronized public void update() {
        double starttime = Timer.getFPGATimestamp();

        
       
    }

    public void aim(){


    }

    public boolean isFinished(){

        return true;
    }


}
