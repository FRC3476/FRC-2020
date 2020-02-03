package frc.subsystem;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
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

Intake instance = new Intake();

//private final TelemetryServer telemetryServer = TelemetryServer.getInstance();
private final Solenoid deploySolenoid;
private final LazyTalonSRX intakeMotor;
private DeployState deployState = DeployState.UNDEPLOY;
private IntakeState intakeState = IntakeState.OFF;



// private double lastDeployCommandTime;

public Intake() {
        super(Constants.intakePeriod);
        deploySolenoid = new Solenoid(Constants.IntakeSolenoidId);
        intakeMotor = new LazyTalonSRX(Constants.IntakeMasterId);
        intakeMotor.setInverted(true);
        intakeMotor.configPeakCurrentLimit(0);
        intakeMotor.configPeakCurrentDuration(0);
        //intakeMotor.configContinousCurrentLimit(25);
        intakeMotor.enableCurrentLimit(true);

}

public DeployState getDeployState() {
        return deployState;
}

public IntakeState getIntakeState() {
        return intakeState;
}

public void setDeployState(final DeployState deployState) {
        synchronized (this) {
                this.deployState = deployState;
        }

        switch (deployState) {
                case DEPLOY:
                        deploySolenoid.set(true);
                        break;
                case UNDEPLOY:
                        deploySolenoid.set(false);
                        break;
        }
        }
public void setIntakeState(IntakeState intakeState) {
        synchronized (this) {
                this.intakeState = intakeState;
        }

        switch(intakeState) {
                case INTAKE:
                        intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPowerIntake);
                case OFF:
                        intakeMotor.set(ControlMode.PercentOutput, 0);
                case EJECT: 
                        intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPowerEject);


        }
}

public void setSpeed(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
}

public double getCurrent() {
        return intakeMotor.getSupplyCurrent();
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
public void logMotorCurrent() {
	// TODO Auto-generated method stub
	
}

@Override
public void update() {	
}
}

