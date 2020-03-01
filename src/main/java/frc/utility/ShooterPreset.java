package frc.utility;


public class ShooterPreset implements Comparable {
    private double hoodEjectA;
    private double flywheelS;
    private double dist;

    ShooterPreset(double hoodEjectAngle, double flywheelSpeed, double distance){
        hoodEjectA = hoodEjectAngle;
        flywheelS = flywheelSpeed;
        dist = distance;
    }

    public double getHoodEjectAngle() {
        return hoodEjectA;
    }

    public double getFlyWheelSpeed(){
        return flywheelS;
    }

    public double getDistance(){
        return dist;
    }

    @Override
    public int compareTo(Object arg0) {
        // TODO Auto-generated method stub
        ShooterPreset a = (ShooterPreset) arg0;
        if (this.getDistance() == a.getDistance()){
            return 0;
        } else if(this.getDistance() > a.getDistance()){
            return 1;
        } else {
            return -1;
        }
        //return Math.copySign(1,  this.getDistance()-a.getDistance());
    }
}