package frc.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;
import frc.utility.JetsonUDP;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
import frc.utility.math.Rotation2D;
import frc.subsystem.*;

@SuppressWarnings("unused")
public class VisionManager extends Subsystem { 
   
    
    boolean waitingOnElevator = false; 
    boolean waitingOnIntake = false;
    long prevTime;
    VisionStatus visionStatus = VisionStatus.IDLE;
    JetsonUDP jetsonUDP = JetsonUDP.getInstance();
    Drive drive = Drive.getInstance();

   
    private static final VisionManager vm = new VisionManager();

   
    public static VisionManager getInstance() {
        return vm;
    }    
    
    public VisionManager() {
        super(Constants.VisionManagerPeriod);
    }

    public enum VisionStatus {
        AIMING, IDLE
    }
    
    @Override
    synchronized public void update() {
        double starttime = Timer.getFPGATimestamp();
        switch(visionStatus){
            case AIMING:
                VisionTarget[] t = jetsonUDP.getTargets();
                if (t != null){
                    double delta_phi = Math.toRadians((2*t[0].x/640.0 - 1) * Constants.CameraXFOV/2) * -1;
                    drive.setRotation(Rotation2D.fromDegrees(drive.getGyroAngle().getDegrees() + delta_phi));
                    
                }
                
                
                break;

            case IDLE:
                break;

        }
        
       
    }

    public void setState(VisionStatus state){
        visionStatus = state;

    }

    public boolean isFinished(){

        return true;
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub

    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub

    }


}
