package frc.subsystem;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Spark;

//for colors Vision failure:
public class BlinkinLED extends Subsystem{
    public enum ActivityCheck{
        ControlPanelActive, VisionActive, Idle
    }

    private ActivityCheck activityCheck = ActivityCheck.Idle;

    private static final BlinkinLED instance = null;//new BlinkinLED(); 
    public static BlinkinLED getInstance() {
        return instance;
    }
    ControlPanel controlPanel = ControlPanel.getInstance();
    Spark spark = new Spark(0);

    public BlinkinLED() {
        super(Constants.BlinkinLEDPeriod);
        controlPanel.getColorSensorData();
    //Saikran said to create instances up here, to create Control Panel Locally(since its final), and to call it.    
    }

    public void ControlPanelCheck(){
        char color = controlPanel.getColorSensorData();

        switch(color){
            case 'B':
                spark.set(0.87);
            break;

            case 'R':
                spark.set(0.61);
            break;

            case 'G':
                spark.set(0.73);
            break;

            case 'Y':
                spark.set(0.69);
            break;
        }
    }


    public void VisionCheck(){
        boolean visionCheck = false;
        //Checks if vision failed or if good
        //If good then set to true if bad then set it to false

        if(visionCheck==true){
            spark.set(0.81);
        }
        else{
            spark.set(0.59);
        }
    }
    @Override
    public void selfTest() {
        // TODO Auto-generated method stub

    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub

    }
@Override
public void start() {
    // TODO Auto-generated method stub
    super.start();
}
    @Override
    public void update() {
        //things classified in here are updated constantly
        switch(activityCheck) {
            case ControlPanelActive:
                ControlPanelCheck();  
            case VisionActive:
                VisionCheck();      
            case Idle:
            spark.set(0.65); 
        }     
    }
}
