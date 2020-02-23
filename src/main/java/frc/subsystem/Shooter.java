package frc.subsystem;

import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.utility.LazyCANSparkMax;
import frc.utility.LazyTalonFX;
import frc.utility.LazyTalonSRX;

import frc.robot.Constants;

public class Shooter extends Subsystem{
    public LazyTalonFX shooterMaster, shooterSlave1, shooterSlave2, shooterSlave3; //Motor controller objects
    private LazyTalonSRX feederMotor;
    private CANPIDController hoodPID;
    private CANEncoder hoodEncoder;
    private LazyCANSparkMax hoodMotor;
    private double tbh = 0;
    private double prevError = 0; 
    private int targetShooterSpeed;
    private int targetHoodPosition;
    private double flywheelError = 0;
    private double prev_error = 0;
    private double shooterOutput = 0;
    private boolean firing = false;
    @SuppressWarnings("unused")
    public static ShooterState shooterState = ShooterState.OFF;

    public Shooter(){
        super(Constants.ShooterPeriod);
        //Shooter Talon ports
        shooterMaster = new LazyTalonFX(Constants.ShooterMasterId);
        shooterSlave1 = new LazyTalonFX(Constants.ShooterSlaveId1);
        shooterSlave2 = new LazyTalonFX(Constants.ShooterSlaveId2);
        shooterSlave3 = new LazyTalonFX(Constants.ShooterSlaveId3);
        feederMotor = new LazyTalonSRX(Constants.FeederMotorId);
        hoodMotor = new LazyCANSparkMax(Constants.HoodMotorId,MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();

        configPID();
    }

    private static final Shooter instance = new Shooter();
	
	public static Shooter getInstance() {
		return instance;
	}

    public enum ShooterState {
        OFF, SPINNING
    }

    public void configPID(){
        //Set forward directions
        shooterMaster.setInverted(true);
        shooterSlave1.setInverted(true);
        shooterSlave2.setInverted(false);
        shooterSlave3.setInverted(false);
        
        //Make slave motors follow the master
        shooterSlave1.follow(shooterMaster);
        shooterSlave2.follow(shooterMaster);
        shooterSlave2.follow(shooterMaster);

        //Config PID constants
        shooterMaster.config_kP(0, Constants.kShooterP, Constants.TimeoutMs);
		shooterMaster.config_kI(0, Constants.kShooterI, Constants.TimeoutMs);
        shooterMaster.config_kD(0, Constants.kShooterD, Constants.TimeoutMs);
        shooterMaster.config_IntegralZone(0, Constants.ShooterIntegralZone, Constants.TimeoutMs);

        feederMotor.config_kP(0, Constants.kFeederP, Constants.TimeoutMs);
        feederMotor.config_kI(0, Constants.kFeederI, Constants.TimeoutMs);
        feederMotor.config_kD(0, Constants.kFeederD, Constants.TimeoutMs);
        feederMotor.config_IntegralZone(0, Constants.FeederIntegralZone);

        hoodPID = hoodMotor.getPIDController();
        hoodPID.setP(Constants.kHoodP, 0);
		hoodPID.setD(Constants.kHoodD, 0);
		hoodPID.setFF(Constants.kHoodF,0);
		hoodPID.setOutputRange(-1, 1);
    }

    public synchronized void setSpeed(int speed) {
        targetShooterSpeed = speed;
        if (speed > 0){
            shooterState = ShooterState.SPINNING;
        } else {
            shooterState = ShooterState.OFF;
        }
    }

    public synchronized void setFiring(boolean fire){
        firing = fire;

    }

    public synchronized void update(){

        System.out.println("out: " + shooterOutput);

        shooterMaster.set(ControlMode.PercentOutput, shooterOutput);
        

        switch(shooterState){
            case SPINNING: 

                flywheelError = targetShooterSpeed - getRPM();                // calculate the error;
                shooterOutput += Constants.TakeBackHalfGain * flywheelError;                     // integrate the output;
                if (flywheelError*prevError<0) { // if zero crossing,
                    shooterOutput = 0.5 * (shooterOutput + tbh);            // then Take Back Half
                    tbh = shooterOutput;                             // update Take Back Half variable
                    prev_error = flywheelError;                       // and save the previous error
                }

                System.out.println("error: " +  flywheelError);
                

                hoodPID.setReference(targetHoodPosition, ControlType.kPosition);
                boolean feederOn = false;
                //check if motor has sped up
                if(Math.abs(flywheelError) <  Constants.ShooterMaxDeviation){
                    double hoodError = targetHoodPosition - hoodEncoder.getPosition();

                    if(Math.abs(hoodError) < Constants.HoodMaxDeviation || true){ // TODO: make hood do things
                        //Hood Ready
                        if(firing){
                            feederOn = true;
                        }
                        

                    }

                }

                if (feederOn){
                    feederMotor.set(ControlMode.PercentOutput, Constants.FeederMotorSpeed);
                } else{
                    feederMotor.set(ControlMode.PercentOutput, 0 );

                }
                break;

            case OFF:
                feederMotor.set(ControlMode.Velocity, 0);
                hoodPID.setReference(0, ControlType.kPosition);

                shooterOutput = 0;
                break;
        }


     }

    public synchronized void setEjectAngle(int angle) {
        //hoodMotor.setPosition();
        targetHoodPosition = angle*Constants.HoodRotationsConversion;
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub

    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub

    }


    private double getRPM(){
        return shooterMaster.getSelectedSensorVelocity() * Constants.ShooterRPMPerTicksPer100ms;


    }

}