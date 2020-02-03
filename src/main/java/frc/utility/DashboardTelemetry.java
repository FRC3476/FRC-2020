package frc.utility;

import com.github.cliftonlabs.json_simple.JsonObject;

import frc.subsystem.Subsystem;

public class DashboardTelemetry {
    Subsystem[] subsystem;
    int[] latestPacket;

    DashboardTelemetry(Subsystem[] subsystem) {
        this.subsystem = subsystem;
        //latestPacket =
    }

    public void send(JsonObject j) {
    
    }

    public void sendAllLogs(int requested) {
        for(Subsystem s : subsystem) {
           // s.logData();
        }
    }

    public void sendLatestLogs(int requested) {
        for(Subsystem s : subsystem) {
           // if(s.dataPacket.length >= 1) 
            
            
            //send(s.dataPacket[s.dataPacket.length - 1]);
           // s.logData();
           
        }
    }

  
    public void update() {

    }
}