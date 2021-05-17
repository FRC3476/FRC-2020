package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyCANSparkMax;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hopper extends Subsystem {
	public enum  FrontMotorState {
		ACTIVE, REVERSE, INACTIVE
	}
	public enum SnailMotorState {
		ACTIVE, REVERSE, INACTIVE 
	}
	
	private static final Hopper instance = new Hopper();
	public static Hopper getInstance() {
		return instance;
	}

	private final LazyTalonSRX FrontHopperMotor;
	private final LazyCANSparkMax SnailMotor;
	private FrontMotorState frontMotorState = FrontMotorState.INACTIVE;
	private SnailMotorState snailMotorState = SnailMotorState.INACTIVE;

	private Hopper() {
		super(Constants.hopperPeriod);
		FrontHopperMotor = new LazyTalonSRX(Constants.FrontHopperMotorId);
		FrontHopperMotor.configContinuousCurrentLimit(20);
		FrontHopperMotor.configPeakCurrentLimit(0);
		FrontHopperMotor.configPeakCurrentDuration(0);
		FrontHopperMotor.enableCurrentLimit(true);
		
		SnailMotor = new LazyCANSparkMax(Constants.SnailMotorId,MotorType.kBrushless);
		//TODO: Set Control Mode
		//SnailMotor.enableCurrentLimit()
	}    

	public FrontMotorState getFrontMotorState() {
		return frontMotorState;
	}

	public SnailMotorState getSnailMotorState() {
		return snailMotorState;
	}

	public double getCurrent() {
		return SnailMotor.getOutputCurrent();
	}

	public void setFrontSpeed(double Frontspeed) {
		FrontHopperMotor.set(ControlMode.PercentOutput, Frontspeed);
	}
	
	public void setSnailSpeed(double Snailspeed) {
		SnailMotor.set(Snailspeed);
	}
	public void setFrontMotorState(final FrontMotorState frontMotorState) {
		synchronized (this) {
			this.frontMotorState = frontMotorState;
		}
		

		switch (frontMotorState) {
				case ACTIVE:
					setFrontSpeed(0.8);

					setFrontSpeed(Constants.HopperFrontMotorSpeed);
					break;
				case INACTIVE:
					setFrontSpeed(0);
					break;
				case REVERSE:
					setFrontSpeed(-Constants.HopperFrontMotorSpeed);
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
						break;
					case INACTIVE: 
						setSnailSpeed(0);
						break;
					case REVERSE:
						setSnailSpeed(-Constants.SlowHopperSnailSpeed);
						break;
			}
			} else {
				switch(snailMotorState) {
					case ACTIVE:
						setSnailSpeed(Constants.HopperSnailSpeed);
						break;
					case INACTIVE: 
						setSnailSpeed(0);
						break;
					case REVERSE:
						setSnailSpeed(-Constants.HopperSnailSpeed);
						break;
			}
			

			
		}
	}


	@Override
	public void selfTest() {
		setFrontMotorState(FrontMotorState.ACTIVE);
		OrangeUtility.sleep((int)(Constants.IntakeOpenTime * 1000));
		setFrontMotorState(FrontMotorState.INACTIVE);
		OrangeUtility.sleep(3000);
		setSnailMotorState(SnailMotorState.ACTIVE, false);
		OrangeUtility.sleep((int)(Constants.IntakeOpenTime * 1000));
		setSnailMotorState(SnailMotorState.INACTIVE, false);
	}

	@Override
	public void logData() {
		
	}

	@Override
	public void update() {
		
	}
}   