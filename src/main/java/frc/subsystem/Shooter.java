package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
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
		shooterSlave1.config_kI(0, Constants.kShooterI, Constants.TimeoutMs);
        shooterSlave2.config_kD(0, Constants.kShooterD, Constants.TimeoutMs);
        shooterSlave3.config_IntegralZone(0, Constants.ShooterIntegralZone, Constants.TimeoutMs);

        feederMotor.config_kP(0, Constants.kFeederP, Constants.TimeoutMs);
        feederMotor.config_kI(0, Constants.kFeederI, Constants.TimeoutMs);
        feederMotor.config_kD(0, Constants.kFeederD, Constants.TimeoutMs);
        feederMotor.config_IntergralZone(0, Constants)

        hoodPID = hoodMotor.getPIDController();
        hoodPID.setP(Constants.kHoodP, 0);
		hoodPID.setD(Constants.kHoodD, 0);
		hoodPID.setFF(Constants.kHoodF,0);
		hoodPID.setOutputRange(-1, 1);
    }

    public static void setSpeed(int speed) {
        shooterMaster.set(ControlMode.Velocity, speed);
    }

    public static void setEjectAngle(int angle) {
        //  hoodMotor.setPosition();
        
    }

}