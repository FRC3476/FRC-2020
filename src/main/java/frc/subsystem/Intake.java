package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import frc.utility.telemetry.TelemetryServer;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {

    public enum DeployState {
	    DEPLOY, UNDEPLOY
    }
	
    public enum IntakeState {
	    INTAKE, OFF, EJECT, SLOW
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
		intakeMotor.configContinuousCurrentLimit(30);
		intakeMotor.enableCurrentLimit(true);
	}

	public synchronized DeployState getDeployState() {
		return deployState;
	}

	public synchronized IntakeState getIntakeState() {
		return intakeState;
	}

	public synchronized void setDeployState(final DeployState deployState) {

		switch (deployState) {
			case DEPLOY:
				//setSpeed(0.64);
				deploySolenoid.set(true);
				if(this.deployState != deployState){
					allowOpenTime = Timer.getFPGATimestamp() + Constants.IntakeOpenTime;
				}
				 
				break;
			case UNDEPLOY:
				//setSpeed(0.64);
				deploySolenoid.set(false);
				intakeState = IntakeState.OFF;
				break;
		}

		
		this.deployState = deployState;
		
	}

	public synchronized void setIntakeState(IntakeState intakeState) {
		this.intakeState = intakeState;
		if((deployState == DeployState.DEPLOY && Timer.getFPGATimestamp() > allowOpenTime)){
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
				case SLOW:
					intakeMotor.set(ControlMode.PercentOutput, 0);
					break;
				default:
					break;
			}
		} else {
			intakeMotor.set(ControlMode.PercentOutput, 0);
		}

	}

	private synchronized void setSpeed(double speed) {
		intakeMotor.set(ControlMode.PercentOutput, speed);
	}

	public synchronized double getCurrent() {
		return intakeMotor.getSupplyCurrent();
	}

	@Override
	public void selfTest() {
		setDeployState(DeployState.DEPLOY);
		OrangeUtility.sleep((int)(Constants.IntakeOpenTime * 1000));
		setIntakeState(IntakeState.INTAKE);
		OrangeUtility.sleep(3000);
		setIntakeState(IntakeState.OFF);
		setDeployState(DeployState.UNDEPLOY);
	}

	@Override
	public void logData() {
		
	}


	public void logMotorCurrent() {
		
	}

	@Override
	public synchronized void update() {
		if(deployState == DeployState.DEPLOY && Timer.getFPGATimestamp() > allowOpenTime){
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
				case SLOW:
					intakeMotor.set(ControlMode.PercentOutput, 0);

					break;
				default:
					break;
			}
		} else {
			intakeMotor.set(ControlMode.PercentOutput, 0);
		}
	}
}


