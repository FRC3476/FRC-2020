package frc.subsystem;

import frc.robot.Constants;
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


	Spark spark = new Spark(0);

	private BlinkinLED() {
		super(Constants.BlinkinLEDPeriod);
	//Saikran said to create instances up here, to create Control Panel Locally(since its final), and to call it.    
	}
	
	public void setColor(double color){
		spark.set(color);
		
	}

	@Override
	public void selfTest() {
		

	}

	@Override
	public void logData() {
		

	}

	@Override
	public void update() {
		

	}

}
