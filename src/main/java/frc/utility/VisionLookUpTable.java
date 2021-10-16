package frc.utility;

import java.util.ArrayList;
import java.util.Collections;

public class VisionLookUpTable{
	ArrayList<ShooterPreset> lookUpTable = new ArrayList<ShooterPreset>();

	private static VisionLookUpTable vt = new VisionLookUpTable();

	public static VisionLookUpTable getInstance(){
		return vt;
	}

	private VisionLookUpTable(){
		lookUpTable.add(new ShooterPreset(47, 4000, 58));
		lookUpTable.add(new ShooterPreset(44, 4300, 73));
		lookUpTable.add(new ShooterPreset(45, 4500, 111));
		lookUpTable.add(new ShooterPreset(39, 5000, 124));
		lookUpTable.add(new ShooterPreset(33.5, 5400, 150));
		lookUpTable.add(new ShooterPreset(35, 5400, 164));
		lookUpTable.add(new ShooterPreset(36, 5600, 173));
		lookUpTable.add(new ShooterPreset(36, 5700, 212));
		lookUpTable.add(new ShooterPreset(36, 6000, 228));
		//lookUpTable.add(new ShooterPreset(38.5, 5700, 223));


		//Good Balls
		// lookUpTable.add(new ShooterPreset(44, 4000, 58));
		// lookUpTable.add(new ShooterPreset(41, 4000, 73));
		// lookUpTable.add(new ShooterPreset(42, 4000, 111));
		// lookUpTable.add(new ShooterPreset(41, 4000, 124));
		// lookUpTable.add(new ShooterPreset(30.5, 5400, 150));
		// lookUpTable.add(new ShooterPreset(33, 5600, 173));
		// lookUpTable.add(new ShooterPreset(33, 5700, 212));

	

		Collections.sort(lookUpTable);

	}

	public ShooterPreset getShooterPreset(double distanceFromTarget){

		if(distanceFromTarget <= lookUpTable.get(0).getDistance()){
			return lookUpTable.get(0);
		}

		for (int i = 1; i < lookUpTable.size(); i++){
			double dist = lookUpTable.get(i).getDistance();

			if(dist == distanceFromTarget){
				return lookUpTable.get(i);

			} else if(dist > distanceFromTarget){
				
				double percentIn = (distanceFromTarget - lookUpTable.get(i-1).getDistance()) / ( lookUpTable.get(i).getDistance() - lookUpTable.get(i-1).getDistance());
				//System.out.println(percentIn + " " + (dist - lookUpTable.get(i-1).getDistance()) + " " + ( lookUpTable.get(i).getDistance() - lookUpTable.get(i-1).getDistance())+ " " + distanceFromTarget);
				
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

