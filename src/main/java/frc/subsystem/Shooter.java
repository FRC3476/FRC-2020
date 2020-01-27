package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.utility.LazyCANSparkMax;

import frc.robot.Constants;

public class Shooter {
    private static TalonFX shooterMaster, shooterSlave1, shooterSlave2, shooterSlave3; //Motor controller objects
    private static CANPIDController hoodPID;
    private static LazyCANSparkMax hoodMotor;
    private static double tbh = 0;
    private static double prevError = 0;

    public Shooter(){
        //Shooter Talon ports
        shooterMaster = new TalonFX(Constants.ShooterMasterId);
        shooterSlave1 = new TalonFX(Constants.ShooterSlaveId1);
        shooterSlave2 = new TalonFX(Constants.ShooterSlaveId2);
        shooterSlave3 = new TalonFX(Constants.ShooterSlaveId3);
        hoodMotor = new LazyCANSparkMax(Constants.HoodMotorId,MotorType.kBrushless);

        configPID();
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
		shooterSlave1.config_kI(0, Constants.kShooterI, Constants.TimeoutMs);
        shooterSlave2.config_kD(0, Constants.kShooterD, Constants.TimeoutMs);
        shooterSlave3.config_IntegralZone(0, Constants.ShooterIntegralZone, Constants.TimeoutMs);

        hoodPID = hoodMotor.getPIDController();
        hoodPID.setP(Constants.kHoodP, 0);
		hoodPID.setD(Constants.kHoodD, 0);
		hoodPID.setFF(Constants.kHoodF,0);
		hoodPID.setOutputRange(-1, 1);
    }

    public static void setSpeed(int speed) {
        shooterMaster.set(ControlMode.Velocity, speed);
    }

    public static void setAngle(int angle) {
        
    }

    public static void TBH(int speed){
        double error = speed - shooterMaster.getSelectedSensorVelocity(0);
        double output = 0;
        output += error*Constants.ShooterGain;

        if(Math.sin(error)*Math.sin(prevError)>0) {
            output = 0.5 * (output+tbh);
            tbh = output;
            prevError = error;
        }

        shooterMaster.set(ControlMode.Velocity, output);
    }

}