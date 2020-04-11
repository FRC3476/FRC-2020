package frc.subsystem;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import com.github.cliftonlabs.json_simple.JsonObject;

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
        if (period != -1)
            new Thread(this).start();
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
        while(signal != ThreadSignal.DEAD) {
            //TODO: Change to old script if it does not work
            // if(signal == ThreadSignal.ALIVE) update();
            // try { 
            //     Thread.sleep(period);
            // } catch(Exception e) {
            //     System.out.println("Thread sleep failing");
            // }
            
            Timer timer = new Timer();
            int begin = 0; //timer begins instantly.
            int timeinterval = period; //timer executes every period.
            timer.scheduleAtFixedRate(new TimerTask() {
                @Override
                public void run() {
                    if(signal == ThreadSignal.ALIVE) update();
                }
            },begin, timeinterval);
        }
    }
}