package frc.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;
import frc.utility.JetsonUDP;
import frc.utility.ShooterPreset;
import frc.utility.Threaded;
import frc.utility.VisionLookUpTable;
import frc.utility.VisionTarget;
import frc.utility.math.Rotation2D;
import frc.subsystem.*;
import frc.subsystem.Drive.DriveState;

@SuppressWarnings("unused")
public class VisionManager extends Subsystem { 
   
    
    boolean waitingOnElevator = false; 
    boolean waitingOnIntake = false;
    long prevTime;
    VisionStatus visionStatus = VisionStatus.IDLE;
    JetsonUDP jetsonUDP = JetsonUDP.getInstance();
    Hopper hopper = Hopper.getInstance();
    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();
    boolean go = false;
    BlinkinLED led;
    public int winStage = 0;
   
    private static final VisionManager vm = new VisionManager();


    public synchronized VisionStatus getState() {
        return visionStatus;
    }
   
    public static VisionManager getInstance() {
        return vm;
    }    
    
    private VisionManager() {
        super(Constants.VisionManagerPeriod);
        led = BlinkinLED.getInstance();
    }

    public enum VisionStatus {
        AIMING, IDLE, WIN
    }

    public void resetWin() {
        winStage = 0;
    }
    
    @Override
    synchronized public void update() {
        double starttime = Timer.getFPGATimestamp();
        VisionTarget[] t;
        switch(visionStatus){
            case AIMING:
                t = jetsonUDP.getTargets();
                double distance = 0;
                if (t != null){
                    double delta_phi = Math.toRadians((2*t[0].x/640.0 - 1) * Constants.CameraXFOV/2) * -1 + Math.toRadians(Constants.VisionXOffset);
                    drive.setRotationTeleop(Rotation2D.fromDegrees(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees() + Math.toDegrees(delta_phi)));
                    //System.out.println("trying to aim, recieving paquet " +  Math.toDegrees(delta_phi));
                    // ShooterPreset d = VisionLookUpTable.getInstance().getShooterPreset(distance);
                    // Shooter.getInstance().setHoodAngle(d.getHoodEjectAngle());
                    // Shooter.getInstance().setSpeed(d.getFlyWheelSpeed());

                } //else System.out.println("recieved null paquet");
                go = go || !drive.isAiming();
                
                if (go){
                    led.setColor(.77);
                } else {
                    led.setColor(-.11);
                }
                
                break;

            case IDLE:
                break;
            
            case WIN:
            //System.out.println("is aiming " + drive.isAiming());
                t = jetsonUDP.getTargets();
                
                if (t != null){
   //                 System.out.println("TR");

                    double delta_phi = Math.toRadians((2*t[0].x/640.0 - 1) * Constants.CameraXFOV/2) * -1 + Math.toRadians(Constants.VisionXOffset);
                    drive.setRotationTeleop(Rotation2D.fromDegrees(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees() + Math.toDegrees(delta_phi)));
                }

                go = go || !drive.isAiming();

                if(go) {

                    shooter.setFiring(true);
                    hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
                    hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
                 //   led.setColor(.77);

                } else {
                    shooter.setFiring(false);
                    hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
                    hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
                   // led.setColor(-.11);

                   System.out.println("not ready: " + go);

                } 
                break;
                
                

        }
        
       
    }

    public void stop() {
        shooter.setFiring(false);
        hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
        hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);
        led.setColor(-.11);
        setState(VisionStatus.IDLE);
    }

    public synchronized void setState(VisionStatus state){
        visionStatus = state;
        if(state == VisionStatus.IDLE)
        {
            drive.driveState = Drive.DriveState.DONE;
            go = false;
            shooter.setFiring(false);
            hopper.setSnailMotorState(Hopper.SnailMotorState.INACTIVE, false);
            hopper.setFrontMotorState(Hopper.FrontMotorState.INACTIVE);

        }



    }

    public synchronized boolean isFinished(){
        return go; //(Drive.getInstance().driveState == DriveState.DONE);
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
