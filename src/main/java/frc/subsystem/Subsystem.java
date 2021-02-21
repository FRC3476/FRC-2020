package frc.subsystem;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.github.cliftonlabs.json_simple.JsonObject;

import edu.wpi.first.wpilibj.Timer;

public abstract class Subsystem implements Runnable {
	int period = 50;

	FileWriter logWriter;
	FileReader logReader; 
	ThreadSignal signal = ThreadSignal.PAUSED;
	public JsonObject latest;
	public String subsystemName;

	public enum ThreadSignal {
		ALIVE, PAUSED, DEAD
	}

	protected static Subsystem instance;

	public Subsystem(int period) {
		this.period = period;
		if (period != -1){
			ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();
			service.scheduleAtFixedRate(this,0, (long) (period), TimeUnit.MILLISECONDS);
		}
			
		try {
			String fileTime = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());
			subsystemName = this.getClass().getSimpleName();
			String fileName = subsystemName+""+fileTime;
			logWriter = new FileWriter(fileName);
			logReader = new FileReader(fileName);
			
		} catch (IOException e) {
			//e.printStackTrace();
		}
	}

	public abstract void selfTest();

	public abstract void logData();

	public static Subsystem getInstance() {
		return instance;
	}
	
	public void pause() {
		signal = ThreadSignal.PAUSED;
	}

	public void kill() {
		signal = ThreadSignal.DEAD;
	}

	public void start() {
		signal = ThreadSignal.ALIVE;
	}

	public abstract void update();

	

	public void run() {
		if(signal != ThreadSignal.DEAD) {
			double startTime = Timer.getFPGATimestamp();
			if(signal == ThreadSignal.ALIVE) update();

			double executionTimeMS = (Timer.getFPGATimestamp()-startTime)*1000;
        	
			
			
		}
	}
}