package frc.utility;

import java.net.DatagramSocket;

import com.github.cliftonlabs.json_simple.JsonObject;

import frc.subsystem.Subsystem;

public class DashboardTelemetry extends Subsystem {
    Subsystem[] subsystem;
    JsonObject[] latestPacket;
    private DatagramSocket dsSocket;

    DashboardTelemetry(int period, Subsystem[] subsystem) {
        super(period);
        this.subsystem = subsystem;
        latestPacket = new JsonObject[subsystem.length];
        
        //latestPacket =
    }

    public void send(JsonObject j) {
        send(j.toJson());
    }

    public void send(String s) {
        
    }

    public static JsonObject getSubsystemLogPacket(Subsystem s) { //return packet with header complete and empty data
        JsonObject packet = new JsonObject();
        JsonObject header = new JsonObject();
        header.put("type", "log");
        header.put("subsystem", s.subsystemName);
        packet.put("header", header);
        return packet;
    }

    public static JsonObject get() {
        return null;
    }
   // public static JsonObject SelfTest 

    public void sendAllLogs(int requested) {
        for(Subsystem s : subsystem) {
            
           // s.logData();
           /*
           for(int i = 0; i < s.dataPacket.length; i++) {
               send(s.dataPacket[i]);
           } */
        }
    }

    public void sendLatestLogs(int requested) {
        for(int i = 0; i < subsystem.length; i++) {
            Subsystem s = subsystem[i];
            if(s.latest != latestPacket[i]) {
                send(s.latest);
                latestPacket[i] = s.latest; 
            }           
        }
    }

    
  
    public void update() {
        
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

}