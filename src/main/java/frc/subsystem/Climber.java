package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.control.RateLimiter;

public class Climber extends Subsystem{

    private LazyTalonFX climberMotor;
    private Solenoid ClimberSolenoid;
    
    private boolean hookingOn = false; 

    private static final Climber instance = new Climber();
	
	public static Climber getInstance() {
		return instance;
	}


    public Climber() {
        super(Constants.ClimberPeriod);
        climberMotor = new LazyTalonFX(Constants.ClimberMotorID);
        climberMotor.config_kP(0, Constants.kClimberP, Constants.TimeoutMs);
        climberMotor.config_kI(0, Constants.kClimberI, Constants.TimeoutMs);
        climberMotor.config_kD(0, Constants.kClimberD, Constants.TimeoutMs);
        climberMotor.config_IntegralZone(0, Constants.ClimberIntergralZone);
        climberMotor.setSelectedSensorPosition(0);
    }



    public void up() {
        //climberMotor.set(ControlMode.Position, Constants.ClimberClimbedHeight);
        if(Math.abs(climberMotor.getSelectedSensorPosition())<Math.abs(Constants.ClimberMaxTarget)){
            climberMotor.set(ControlMode.PercentOutput, -.8);
        } else{
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
        

        

        
    }

    public void down(){
        if(Math.abs(climberMotor.getSelectedSensorPosition())<Math.abs(Constants.ClimberClimbedHeight))
        climberMotor.set(ControlMode.PercentOutput, -.8);
    }

    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
        
    }

    void hookOn() {
        hookingOn = true;

        
    }

    public void reset() {
        climberMotor.set(ControlMode.PercentOutput, .2);
        
    }



    @Override
    public void selfTest() {


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
        System.out.println(climberMotor.getSelectedSensorPosition());
    }
    
}




