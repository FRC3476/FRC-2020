package frc.subsystem;
import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class Hopper extends Subsystem {
	public enum  FrontMotorState {
		ACTIVE, REVERSE, INACTIVE
	}
	public enum SnailMotorState {
		ACTIVE, REVERSE, INACTIVE, ANTIJAM 
	}
	
	private static final Hopper instance = new Hopper();
	public static Hopper getInstance() {
		return instance;
	}

	private final LazyTalonSRX FrontHopperMotor;
	private final LazyTalonSRX SnailMotor;
	private FrontMotorState frontMotorState = FrontMotorState.INACTIVE;
	private SnailMotorState snailMotorState = SnailMotorState.INACTIVE;

	private Hopper() {
		super(Constants.hopperPeriod);
		FrontHopperMotor = new LazyTalonSRX(Constants.FrontHopperMotorId);
		FrontHopperMotor.configContinuousCurrentLimit(3 );
		FrontHopperMotor.configPeakCurrentLimit(0);
		FrontHopperMotor.configPeakCurrentDuration(0);
		FrontHopperMotor.enableCurrentLimit(true);
		
		SnailMotor = new LazyTalonSRX(Constants.SnailMotorId);
		System.out.println("Current limit error " + SnailMotor.configContinuousCurrentLimit(25, 50));
		SnailMotor.configPeakCurrentLimit(0);
		SnailMotor.configPeakCurrentDuration(0);
		SnailMotor.enableCurrentLimit(true);
		//SnailMotor.enableCurrentLimit()
		currentLimitEnabled = true;
	}    


	boolean currentLimitEnabled = true;
	private void disableCurrentLimit(){
		if(currentLimitEnabled){
			FrontHopperMotor.configContinuousCurrentLimit(40);
			FrontHopperMotor.configPeakCurrentLimit(0);
			FrontHopperMotor.configPeakCurrentDuration(0);
			FrontHopperMotor.enableCurrentLimit(true);
			
			SnailMotor.configContinuousCurrentLimit(40);
			SnailMotor.configPeakCurrentLimit(0);
			SnailMotor.configPeakCurrentDuration(0);
			SnailMotor.enableCurrentLimit(true);

			currentLimitEnabled = false;
		}
	}

	public void enableCurrentLimit(){
		if(!currentLimitEnabled){
			FrontHopperMotor.configContinuousCurrentLimit(3);
			FrontHopperMotor.configPeakCurrentLimit(0);
			FrontHopperMotor.configPeakCurrentDuration(0);
			FrontHopperMotor.enableCurrentLimit(true);
			
			SnailMotor.configContinuousCurrentLimit(15);
			SnailMotor.configPeakCurrentLimit(0);
			SnailMotor.configPeakCurrentDuration(0);
			SnailMotor.enableCurrentLimit(true);
			currentLimitEnabled = true; 
		}
	}

	public FrontMotorState getFrontMotorState() {
		return frontMotorState;
	}

	public SnailMotorState getSnailMotorState() {
		return snailMotorState;
	}

	public double getCurrent() {
		return SnailMotor.getSupplyCurrent();
	}

	public void setFrontSpeed(double Frontspeed) {
		FrontHopperMotor.set(ControlMode.PercentOutput, Frontspeed);
	}
	
	public void setSnailSpeed(double Snailspeed) {
		SnailMotor.set(ControlMode.PercentOutput, Snailspeed);
	}
	public void setFrontMotorState(final FrontMotorState frontMotorState) {
		synchronized (this) {
			this.frontMotorState = frontMotorState;
		}
		

		switch (frontMotorState) {
				case ACTIVE:
					//setFrontSpeed(0.8);
					setFrontSpeed(Constants.HopperFrontMotorSpeed);
					enableCurrentLimit();
					break;
				case INACTIVE:
					setFrontSpeed(0);
					enableCurrentLimit();
					break;
				case REVERSE:
					setFrontSpeed(-Constants.HopperFrontMotorSpeed);
					disableCurrentLimit();
					break;
		}
	}

	public void setSnailMotorState(SnailMotorState snailMotorState, boolean slow) {
			synchronized (this) {
					this.snailMotorState = snailMotorState;
			}
			if(slow){
				switch(snailMotorState) {
					case ACTIVE:
						setSnailSpeed(Constants.SlowHopperSnailSpeed);
						enableCurrentLimit();
						break;
					case INACTIVE: 
						setSnailSpeed(0);
						enableCurrentLimit();
						break;
					case REVERSE:
						setSnailSpeed(-Constants.SlowHopperSnailSpeed);
						disableCurrentLimit();
						break;
					case ANTIJAM:
						if((((int) Timer.getFPGATimestamp()*4) % 3) < 2){
							setSnailSpeed(Constants.SlowHopperSnailSpeed);
						} else {
							setSnailSpeed(0);
						}
						
						disableCurrentLimit();
						break;
			}
			} else {
				switch(snailMotorState) {
					case ACTIVE:
						setSnailSpeed(Constants.HopperSnailSpeed);
						enableCurrentLimit();
						break;
					case INACTIVE: 
						setSnailSpeed(0);
						enableCurrentLimit();
						break;
					case REVERSE:
						setSnailSpeed(-Constants.HopperSnailSpeed);
						disableCurrentLimit();
						break;
					case ANTIJAM:
						if(((Timer.getFPGATimestamp()*10) % 3) < 2){
							setSnailSpeed(Constants.SlowHopperSnailSpeed);
						} else {
							setSnailSpeed(0);
						}
						
						disableCurrentLimit();
						break;
			}
			

			
		}
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