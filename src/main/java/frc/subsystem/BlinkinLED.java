package frc.subsystem;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;

//for colors Vision failure:
public class BlinkinLED extends Subsystem {
    public enum ActivityCheck {
        ControlPanelActive, VisionActive, Idle
    }

    //private ActivityCheck activityCheck = ActivityCheck.Idle;

    private static final BlinkinLED instance = new BlinkinLED();

    public static BlinkinLED getInstance() {
        return instance;
    }

    ControlPanel controlPanel = ControlPanel.getInstance();

    PWMSparkMax spark = new PWMSparkMax(0);

    public BlinkinLED() {
        super(Constants.BlinkinLEDPeriod);
    //Saikran said to create instances up here, to create Control Panel Locally(since its final), and to call it.    
    }
    
    public void setColor(double color){
        spark.set(color);
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
    public void update() {
        // TODO Auto-generated method stub

    }

}
