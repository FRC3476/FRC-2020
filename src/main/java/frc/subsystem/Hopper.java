package frc.subsystem;
import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Hopper extends Subsystem {
    public enum  FrontMotorState {
        ACTIVE, REVERSE, INACTIVE
    }
    public enum SnailMotorState {
        ACTIVE, REVERSE, INACTIVE 
    }
    
    public static final Hopper instance = new Hopper();
    public static Hopper getInstance() {
        return instance;
}

private final LazyTalonSRX FrontHopperMotor;
private final LazyTalonSRX SnailMotor;
private FrontMotorState frontMotorState = FrontMotorState.INACTIVE;
private SnailMotorState snailMotorState = SnailMotorState.INACTIVE;

public Hopper() {
    super(Constants.hopperPeriod);
    FrontHopperMotor = new LazyTalonSRX(Constants.FrontHopperMotorId);
    FrontHopperMotor.configContinuousCurrentLimit(20);
    FrontHopperMotor.configPeakCurrentLimit(0);
    FrontHopperMotor.configPeakCurrentDuration(0);
    FrontHopperMotor.enableCurrentLimit(true);
    
    SnailMotor = new LazyTalonSRX(Constants.SnailMotorId);
    SnailMotor.configContinuousCurrentLimit(20);
    SnailMotor.configPeakCurrentLimit(0);
    SnailMotor.configPeakCurrentDuration(0);
    SnailMotor.enableCurrentLimit(true);
    }    

    public FrontMotorState getFrontMotorState() {
        return frontMotorState;
}

    public SnailMotorState getSnailMotorState() {
        return snailMotorState;
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
                    setFrontSpeed(0.8);
                    break;
                case INACTIVE:
                    setFrontSpeed(0); //random numbers
                    break;
                case REVERSE:
                    setFrontSpeed(-0.8);
                    break;
        }
    }

    public void setSnailMotorState(SnailMotorState snailMotorState) {
            synchronized (this) {
                    this.snailMotorState = snailMotorState;
            }
            
            switch(snailMotorState) {
                    case ACTIVE:
                        setSnailSpeed(0.8);
                        break;
                    case INACTIVE: //random numbers
                        setSnailSpeed(0);
                        break;
                    case REVERSE:
                        setSnailSpeed(-0.8);
                        break;
            }
    }


	@Override
	public void selfTest() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void logData() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void update() {
		// TODO Auto-generated method stub
		
    }
}   