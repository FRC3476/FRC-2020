package frc.utility;

import java.io.IOException;
import java.io.Writer;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.util.JSONPObject;
import com.github.cliftonlabs.json_simple.JsonObject;

import edu.wpi.first.wpilibj.Timer;
import frc.auton.ShootAndMove;
import frc.subsystem.Subsystem;

public class DashboardTelemetry extends Subsystem {
    Subsystem[] subsystem;
    JsonObject[] latestPacket;
    private DatagramSocket daSocket;
    private DatagramPacket daPacket;
    private String clientIP = "192.168.56.1";
    private int clientPort = 3004;
    private String robotIP = "localhost";
    private boolean connected = false;
    private JsonConverter jsonConverter;

    private double prevUpdateTime = Timer.getFPGATimestamp();

    public static DashboardTelemetry instance;

    public static DashboardTelemetry getInstance(int period, Subsystem[] subsystem){
        instance = new DashboardTelemetry(period, subsystem);
        return instance;
    }
    

    DashboardTelemetry(int period, Subsystem[] subsystem) {
        super(period);
        this.subsystem = subsystem;
        latestPacket = new JsonObject[subsystem.length];
        jsonConverter = new JsonConverter();

        try {
            daSocket = new DatagramSocket(3003);
        } catch (SocketException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        // latestPacket =
    }

    public void send(JsonObject j) {
        try {
            send(j.toJson());
        } catch (UnknownHostException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void send(String s) throws UnknownHostException {
        InetAddress clientAddress = InetAddress.getByName(clientIP);
        byte[] buffer = s.getBytes();
        DatagramPacket response = new DatagramPacket(buffer, buffer.length, clientAddress, clientPort);
        try {
            daSocket.send(response);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static JsonObject getSubsystemLogPacket(Subsystem s) { // return packet with header complete and empty data
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
        String bigLog = "log[";
        for (Subsystem s : subsystem) {

            // s.logData();
            /*
             * for(int i = 0; i < s.dataPacket.length; i++) { send(s.dataPacket[i]); }
             */

        }
    }

    public void sendLiveLogs() {
        System.out.println("Running1");

        JsonObject packet = new JsonObject();
        JsonObject header = new JsonObject();
        JsonObject data = new JsonObject();

        for (int i = 0; i < subsystem.length; i++) {
            Subsystem s = subsystem[i];
            if (s.latest != latestPacket[i]) {

                header.put("type", "subsystemData");
                data.put(s.subsystemName, s.latest.get("state"));

            }
        }

        packet.put("data", data);
        send(packet);
        System.out.println("Running");

        // latestPacket[i] = s.latest;
    }

    public void shoutIP() {
        // byte[] b = ("0"+InetAddress.getLocalHost().getHostAddress()).getBytes();

        JsonObject packet = new JsonObject();
        JsonObject header = new JsonObject();
        JsonObject data = new JsonObject();
        header.put("type", "robot-ip");
        data.put("robotIP", robotIP);
        packet.put("header", header);
        packet.put("data", data);

        new Thread(new Runnable() {

            public void run() {

                while (!connected) {
                    prevUpdateTime = Timer.getFPGATimestamp();
                    try {
                        // DatagramPacket ok = new DatagramPacket(b, b.length);
                        send(packet);
                        Thread.sleep(1000);
                    } catch (Exception e) {

                        System.out.println(e);
                    }
                    ;
                }
            }
        }).start();
    }

    private Thread t2 = new Thread(new Runnable(){
        public void run(){
            while(connected){
                listenForCommand();
            }
        }
    });
    public void listenForOk() throws IOException {
        byte[] b = new byte[256];
        DatagramPacket packet = new DatagramPacket(b, b.length);
        daSocket.receive(packet);
        String message = new String(b, packet.getLength());

        if (message.equals("\"{header\":{\"type\":\"ok\"}}")) {
            connected = true;
        }

    }

    public void listenForCommand() {
        byte[] b = new byte[256];
        DatagramPacket packet = new DatagramPacket(b, b.length);
        try {
            daSocket.receive(packet);
        } catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
        String message = new String(b);
        Map<String, String> map;
        try {
            map = jsonConverter.getJsonObject(message);
            //JsonObject jsonObj = new JsonObject(map);
            
            if(map.get("type").equals("test")){
                if(isChecking(map,"intake")){
                    subsystem[0].selfTest();
                }
                if(isChecking(map,"shooter")){
                        subsystem[1].selfTest();
                }
                if(isChecking(map,"drivebase")){
                    subsystem[2].selfTest();
                }
                if(isChecking(map,"hopper")){
                    subsystem[3].selfTest();
                }
                if(isChecking(map,"climber")){
                    subsystem[4].selfTest();
                }
                if(isChecking(map,"controlPanel")){
                    subsystem[5].selfTest();
                }
            }

            else if(map.get("type").equals("download")){
                String date = map.get("latestDate");
            }

           else if(map.get("type").equals("autoSelect")){
                switch(map.get("selectedAuto")){
                    case "MiddleScoring":

                    break;

                    case "ShootAndMove":
                    break;
                    
                    case "ShootOnly":
                    break;

                    case "TenBall":
                    break;

                    case "TrenchBlue":
                    break;
                }
            }


        } catch (JsonProcessingException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }


    }

    public boolean isChecking(Map<String,String> map, String subsystem){
        if (map.get(subsystem).equals("true")){
            return true;
        }
        return false;
    }

    public void update() {
        System.out.println("Update started");

        if(!connected){
            try {
                //System.out.println("Not Connected");
                shoutIP();
                listenForOk();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        System.out.println("CONNECTED!");
        sendLiveLogs();
        //listenForCommand();
        /*if(!t2.isAlive()&&connected){
            t2.start();
        }*/
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