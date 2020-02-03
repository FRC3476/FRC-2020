package frc.subsystem;

import com.github.cliftonlabs.json_simple.JsonObject;

public abstract class Subsystem implements Runnable {
    int period = 50;
    ThreadSignal signal = ThreadSignal.PAUSED;
    public JsonObject dataPacket[];

    public enum ThreadSignal {
		ALIVE, PAUSED, DEAD
	}

    protected static Subsystem instance;

    Subsystem(int period) {
        if(period != -1) new Thread(this).start();
    }

    public abstract void selfTest();

    public abstract void logData();

    public abstract void logMotorCurrent();

    public static Subsystem getInstance() {
        return instance;
    }
    
    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public abstract void update();

    

    public void run() {
        while(signal != ThreadSignal.DEAD) {
            if(signal == ThreadSignal.ALIVE) update();
            try { 
                Thread.sleep(period);
            } catch(Exception e) {
                System.out.println("Thread sleep failing");
            }
        }
    }
}