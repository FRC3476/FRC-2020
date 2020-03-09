package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.github.cliftonlabs.json_simple.JsonObject;

import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.control.RateLimiter;

import edu.wpi.first.wpilibj.Timer;

public class Climber extends Subsystem{

    private LazyTalonFX ClimberMotor;
    private Solenoid ClimberSolenoid;
    private RateLimiter limiter;
    private boolean hookingOn = false; 

    private static final Climber instance = null ;//new Climber();
	
	public static Climber getInstance() {
		return instance;
	}


    Climber() {
        super(Constants.ClimberPeriod);
        ClimberMotor = new LazyTalonFX(Constants.ClimberMotorID);
        ClimberMotor.config_kP(0, Constants.kClimberP, Constants.TimeoutMs);
        ClimberMotor.config_kI(0, Constants.kClimberI, Constants.TimeoutMs);
        ClimberMotor.config_kD(0, Constants.kClimberD, Constants.TimeoutMs);
        ClimberMotor.config_IntegralZone(0, Constants.ClimberIntergralZone);
        limiter = new RateLimiter(Constants.ClimberHookingMaxVel, Constants.ClimberHookingMaxAcel);
        ClimberSolenoid.set(false);

        ClimberSolenoid = new Solenoid(Constants.ClimberSolenoidID);
    }



    void armUp() {
        ClimberSolenoid.set(false);
        ClimberMotor.set(ControlMode.Position, Constants.ClimberClimbedHeight);
        
    }

    void hookOn() {
        ClimberSolenoid.set(true);
        hookingOn = true;

        
    }

    void reset() {
        ClimberSolenoid.set(false);
        ClimberMotor.set(ControlMode.Position, 0);
        
    }

    @Override
    public void selfTest() {
        selfTestStart=Timer.getFPGATimestamp();
        testing=true;
        ClimberMotor.set(ControlMode.PercentOutput,20);

    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub

    }

    public void logMotorCurrent() {
        // TODO Auto-generated method stub

    }

    @Override
    public void update() {
        latest = new JsonObject();

        if (hookingOn){
            ClimberMotor.set(ControlMode.Velocity, limiter.update(Constants.ClimberClimbedHeight));
            latest.put("state", "RUNNING");
        } else {
            latest.put("state", "OFF");
        }


        //Update stuff for self test
        if(testing==true){
            if(Timer.getFPGATimestamp()-selfTestStart>2000){
                testing=false;
                ClimberMotor.set(ControlMode.PercentOutput,0);
            }
        }

    }
    
}




