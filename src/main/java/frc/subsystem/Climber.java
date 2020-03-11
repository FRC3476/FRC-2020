package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.control.RateLimiter;

public class Climber extends Subsystem{

    private LazyTalonFX climberMotor;
    private Solenoid ClimberSolenoid;
    
    private boolean hookingOn = false; 

    private boolean startClimb = false; 

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
        climberMotor.setNeutralMode(NeutralMode.Coast);
        climberMotor.setSelectedSensorPosition(0);
    }



    public void up() {
        //climberMotor.set(ControlMode.Position, Constants.ClimberClimbedHeight);
        if(Math.abs(climberMotor.getSelectedSensorPosition())<Math.abs(Constants.ClimberMaxTarget)){
            climberMotor.set(ControlMode.PercentOutput, Constants.climberClimbSpeed);
            startClimb = true; 
        } else{
            stop();
        }
        


        

        
    }

    public void down(){
        if(Math.abs(climberMotor.getSelectedSensorPosition())<Math.abs(Constants.ClimberClimbedHeight) &&
            Math.abs(climberMotor.getSelectedSensorPosition()) > Math.abs(Constants.ClimberMaxTarget)*0.8 ){
                climberMotor.set(ControlMode.PercentOutput, Constants.climberClimbSpeed);
            } else {
                stop();
            }
            
    }

    public void stop() {
        if(!startClimb) climberMotor.set(ControlMode.PercentOutput, Constants.climberIdleSpeed);
        else climberMotor.set(ControlMode.PercentOutput, 0);
        
    }

    void hookOn() {
        hookingOn = true;

        
    }

    public void reset() {
// startClimb = false; 
        climberMotor.set(ControlMode.PercentOutput, .2);
        
    }

    public double getCurrent()
    {
        return climberMotor.getStatorCurrent();
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
      //  System.out.println(climberMotor.getSelectedSensorPosition());
    }
    
}




