package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.github.cliftonlabs.json_simple.JsonObject;

//import frc.utility.telemetry.TelemetryServer;
import frc.utility.LazyTalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {

    public enum DeployState {
            DEPLOY, UNDEPLOY
    }
    public enum IntakeState {
            INTAKE, OFF, EJECT
    }

public static final Intake instance = new Intake();

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

public Intake() {
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
	// TODO Auto-generated method stub
        selfTestStart=Timer.getFPGATimestamp();
        testing=true;
        intakeState = intakeState.INTAKE;
}






@Override
public void logData() {
	// TODO Auto-generated method stub
	
}


public void logMotorCurrent() {
	// TODO Auto-generated method stub
	
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
                //Sets the latest state
                latest = new JsonObject();
                latest.put("state", intakeState.toString());
        
                //Update stuff for self test
                if(testing==true){
                    if(Timer.getFPGATimestamp()-selfTestStart>2000){
                        testing=false;
                        //intakeState=IntakeState.OFF;
                        setIntakeState(IntakeState.OFF);
                    }
                }
}
}