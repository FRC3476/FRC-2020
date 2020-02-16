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

public class Shooter {
    private static LazyTalonFX shooterMaster, shooterSlave1, shooterSlave2, shooterSlave3; //Motor controller objects
    private static LazyTalonSRX feederMotor;
    private static CANPIDController hoodPID;
    private static CANEncoder hoodEncoder;
    private static LazyCANSparkMax hoodMotor;
    private static double tbh = 0;
    private static double prevError = 0; 
    private static int targetShooterSpeed;
    private static int targetHoodPosition;
    private static double error = 0;
    private static double prev_error = 0;
    private static double shooterOutput = 0;

    public static ShooterState shooterState = ShooterState.OFF;

    public Shooter(){
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
        OFF, SHOOTING
    }

    public void configPID(){
        //Set forward directions
        shooterMaster.setInverted(false);
        shooterSlave1.setInverted(false);
        shooterSlave2.setInverted(true);
        shooterSlave3.setInverted(true);
        
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

    public static void setSpeed(int speed) {
        targetShooterSpeed = speed;
        prev_error = 
        error = targetShooterSpeed - shooterMaster.getSelectedSensorVelocity(); // calculate the error;    
        prev_error = error;           
        shooterOutput += Constants.TakeBackHalfGain * error;                     // integrate the output;
        if (error*prevError<0) { // if zero crossing,
            shooterOutput = 0.5 * (shooterOutput + tbh);            // then Take Back Half
            tbh = shooterOutput;                             // update Take Back Half variable
            prev_error = error;                       // and save the previous error
        }

        shooterMaster.set(ControlMode.PercentOutput, shooterOutput);
    }

     public static void Shoot(){
        shooterState = ShooterState.SHOOTING;

     }

     public static void StopShoot(){        
        shooterState = ShooterState.OFF;
    }


    public void update(){

        error = targetShooterSpeed - shooterMaster.getSelectedSensorVelocity();                // calculate the error;
            shooterOutput += Constants.TakeBackHalfGain * error;                     // integrate the output;
            if (error*prevError<0) { // if zero crossing,
                shooterOutput = 0.5 * (shooterOutput + tbh);            // then Take Back Half
                tbh = shooterOutput;                             // update Take Back Half variable
                prev_error = error;                       // and save the previous error
            }

            shooterMaster.set(ControlMode.PercentOutput, shooterOutput);


        if (shooterState == ShooterState.SHOOTING) {

            

            hoodPID.setReference(targetHoodPosition, ControlType.kPosition);
            
            //check if motor has sped up
            if(!(shooterMaster.getSelectedSensorVelocity() < targetShooterSpeed - Constants.ShooterMaxDeviation||shooterMaster.getSelectedSensorVelocity() > targetShooterSpeed + Constants.ShooterMaxDeviation )){
                if(!(hoodEncoder.getPosition() <targetHoodPosition - Constants.HoodMaxDeviation || hoodEncoder.getPosition()> targetHoodPosition + Constants.HoodMaxDeviation)){
                    feederMotor.set(ControlMode.Velocity, Constants.FeederMotorSpeed);

                } else {
                    feederMotor.set(ControlMode.Velocity, Constants.FeederMotorSpeed);

                }

            } else {
                System.out.println("Shooter above max RPM deviation");
                feederMotor.set(ControlMode.Velocity,0);
            }
        } else{

            feederMotor.set(ControlMode.Velocity, 0);
            hoodPID.setReference(0, ControlType.kPosition);
            
        }
            


     }

    public static void setEjectAngle(int angle) {
        //hoodMotor.setPosition();
        targetHoodPosition = angle*Constants.HoodRotationsConversion;
    }

}