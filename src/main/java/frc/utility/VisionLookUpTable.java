package frc.utility;

import java.util.ArrayList;
import java.util.Collections;

import frc.utility.*;

public class VisionLookUpTable{
    ArrayList<ShooterPreset> lookUpTable = new ArrayList<ShooterPreset>();

    VisionLookUpTable(){
        lookUpTable.add(new ShooterPreset(10, 2000, 10));
        lookUpTable.add(new ShooterPreset(20, 3000, 20));
        lookUpTable.add(new ShooterPreset(30, 4000, 30));
        lookUpTable.add(new ShooterPreset(40, 5000, 40));
        

        Collections.sort(lookUpTable);

    }

    ShooterPreset getShooterPreset(double DistanceFromTarget){

        if(lookUpTable.get(0).getDistance() > DistanceFromTarget){
            return lookUpTable.get(0);
        }

        for (int i = 0; i < lookUpTable.size(); i++){
            double dist = lookUpTable.get(i).getDistance();

            if(dist == DistanceFromTarget){
                return lookUpTable.get(i);

            } else if(dist > DistanceFromTarget){

                double percentIn = (dist - lookUpTable.get(i-1).getDistance()) / ( lookUpTable.get(i).getDistance() - lookUpTable.get(i-1).getDistance());

                return interpolateShooterPreset(lookUpTable.get(i-1), lookUpTable.get(i), percentIn);
            }
            
        }

        return lookUpTable.get(lookUpTable.size()-1);



    }
    //interpolate() is startValue + (endValue - startValue) * fraction
    

    private ShooterPreset interpolateShooterPreset(ShooterPreset startValue, ShooterPreset endValue, double percentIn) {
        double flywheelSpeed = startValue.getFlyWheelSpeed() + (endValue.getFlyWheelSpeed() - startValue.getFlyWheelSpeed()) * percentIn;
        double hoodPosition = startValue.getHoodEjectAngle() + (endValue.getHoodEjectAngle() - startValue.getHoodEjectAngle()) * percentIn;
        double distance = startValue.getDistance() + (endValue.getDistance() - startValue.getDistance()) * percentIn;

        return new ShooterPreset(hoodPosition, flywheelSpeed, distance);
    }


}

