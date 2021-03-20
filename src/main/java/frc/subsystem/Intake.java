package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import frc.utility.telemetry.TelemetryServer;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {

    public enum DeployState {
	    DEPLOY, UNDEPLOY
    }
    public enum IntakeState {
	    INTAKE, OFF, EJECT
    }


private static final Intake instance = new Intake();
public static Intake getInstance() {
	return instance;
}

//private final TelemetryServer telemetryServer = TelemetryServer.getInstance();
private final Solenoid deploySolenoid;
private final LazyTalonSRX intakeMotor;
private DeployState deployState = DeployState.UNDEPLOY;
private IntakeState intakeState = IntakeState.OFF;
private double allowOpenTime = 0;



// private double lastDeployCommandTime;

private Intake() {
	super(Constants.intakePeriod);
	deploySolenoid = new Solenoid(Constants.IntakeSolenoidId);
	intakeMotor = new LazyTalonSRX(Constants.IntakeMasterId);
	intakeMotor.setInverted(true);
	intakeMotor.configPeakCurrentLimit(0);
	intakeMotor.configPeakCurrentDuration(0);
	intakeMotor.configContinuousCurrentLimit(40);
	intakeMotor.enableCurrentLimit(true);
}

public synchronized DeployState getDeployState() {
	return deployState;
}

public synchronized IntakeState getIntakeState() {
	return intakeState;
}

public void setDeployState(final DeployState deployState) {
	synchronized (this) {
		this.deployState = deployState;
	}
	

	switch (deployState) {
		case DEPLOY:
			//setSpeed(0.64);
			deploySolenoid.set(true);
			allowOpenTime = Timer.getFPGATimestamp() + Constants.IntakeOpenTime;
			break;
		case UNDEPLOY:
			//setSpeed(0.64);
			deploySolenoid.set(false);
			intakeState = IntakeState.OFF;
			break;
	}
	}
public synchronized void setIntakeState(IntakeState intakeState) {
	this.intakeState = intakeState;
	switch(intakeState) {
		case EJECT:
			intakeMotor.set(ControlMode.PercentOutput, -Constants.IntakeMotorPower);
			break;
		case OFF:
			intakeMotor.set(ControlMode.PercentOutput, 0.0);
			break;
		case INTAKE:
			intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPower);  
			break;

	}

}

public synchronized void setSpeed(double speed) {
	intakeMotor.set(ControlMode.PercentOutput, speed);
}

public synchronized double getCurrent() {
	return intakeMotor.getSupplyCurrent();
}

@Override
public void selfTest() {
	deploySolenoid = new Solenoid(Constants.IntakeSolenoidId);
	deploySolenoid.set(true);
	OrangeUtility.sleep(3000);
	deploySolenoid.set(false);
}

@Override
public void logData() {
	
}


public void logMotorCurrent() {
	
}

@Override
public synchronized void update() {	
	/*
	switch(intakeState) {
		case INTAKE:
			if (allowOpenTime<Timer.getFPGATimestamp()){
				intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPowerIntake);
			}   
		case OFF:
			intakeMotor.set(ControlMode.PercentOutput, 0);
		case EJECT: 
			if (allowOpenTime<Timer.getFPGATimestamp()){
				intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPowerEject);
			}

	}*/
}
}