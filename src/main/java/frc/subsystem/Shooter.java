package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.utility.LazyCANSparkMax;
import frc.utility.LazyTalonFX;
import frc.utility.LazyTalonSRX;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import frc.utility.ShooterPreset;
import frc.utility.VisionLookUpTable;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class Shooter extends Subsystem {
	public LazyTalonFX shooterMaster, shooterSlave1, shooterSlave2, shooterSlave3; // Motor controller objects
	private LazyTalonSRX feederMotor;
	private CANPIDController hoodPID;
	private CANEncoder hoodEncoder;
	private LazyCANSparkMax hoodMotor;
	DigitalInput homeSwitch;
	private double tbh = 0;
	private double prevError = 0;
	private double targetShooterSpeed;
	private double targetHoodPosition;
	private double flywheelError = 0;
	private double prev_error = 0;
	private boolean firing = false;
	private double hoodHomeStart;
	private boolean timerStarted = false;
	private static ShooterState shooterState = ShooterState.OFF;
	private VisionLookUpTable visionLookUpTable;
	Limelight limelight;
	// private SynchronousPid turnPID;

	private Shooter() {
		super(Constants.ShooterPeriod);
		// Shooter Talon ports
		shooterMaster = new LazyTalonFX(Constants.ShooterMasterId);
		shooterSlave1 = new LazyTalonFX(Constants.ShooterSlaveId1);
		shooterSlave2 = new LazyTalonFX(Constants.ShooterSlaveId2);
		shooterSlave3 = new LazyTalonFX(Constants.ShooterSlaveId3);
		feederMotor = new LazyTalonSRX(Constants.FeederMotorId);
		hoodMotor = new LazyCANSparkMax(Constants.HoodMotorId, MotorType.kBrushless);
		hoodEncoder = hoodMotor.getEncoder();
		homeSwitch = new DigitalInput(Constants.HomeSwitchId);
		visionLookUpTable = VisionLookUpTable.getInstance();
		limelight = Limelight.getInstance();

		hoodMotor.setInverted(Constants.HoodMotorDirection);
		configPID();
	}

	private static final Shooter instance = new Shooter();

	public static Shooter getInstance() {
		return instance;
	}

	public enum ShooterState {
		OFF, SPINNING, HOMING, EJECT
	}

	public void configPID() {
		// Set forward directions
		shooterMaster.setInverted(true);
		shooterSlave1.setInverted(true);
		shooterSlave2.setInverted(false);
		shooterSlave3.setInverted(false);

		// Make slave motors follow the master
		shooterSlave1.follow(shooterMaster);
		shooterSlave2.follow(shooterMaster);
		shooterSlave3.follow(shooterMaster);

		// Config PID constansts
		// unused
		shooterMaster.config_kP(0, Constants.kShooterP, Constants.TimeoutMs);
		shooterMaster.config_kI(0, Constants.kShooterI, Constants.TimeoutMs);
		shooterMaster.config_kD(0, Constants.kShooterD, Constants.TimeoutMs);
		shooterMaster.config_kF(0, Constants.kShooterF, Constants.TimeoutMs);
		shooterMaster.config_IntegralZone(0, Constants.ShooterIntegralZone, Constants.TimeoutMs);

		feederMotor.config_kP(0, Constants.kFeederP, Constants.TimeoutMs);
		feederMotor.config_kI(0, Constants.kFeederI, Constants.TimeoutMs);
		feederMotor.config_kD(0, Constants.kFeederD, Constants.TimeoutMs);
		feederMotor.config_IntegralZone(0, Constants.FeederIntegralZone);
		feederMotor.setInverted(false);

		hoodMotor.setSmartCurrentLimit(15);
		shooterMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 0, 0));
		shooterSlave1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 0, 0));
		shooterSlave2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 0, 0));
		shooterSlave3.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 0, 0));

		shooterMaster.setNeutralMode(NeutralMode.Coast);
		shooterSlave1.setNeutralMode(NeutralMode.Coast);
		shooterSlave2.setNeutralMode(NeutralMode.Coast);
		shooterSlave3.setNeutralMode(NeutralMode.Coast);

		hoodPID = hoodMotor.getPIDController();
		hoodPID.setP(Constants.kHoodP, 0);
		hoodPID.setD(Constants.kHoodD, 0);
		hoodPID.setFF(Constants.kHoodF, 0);
		hoodPID.setOutputRange(-Constants.HoodPIDSpeedMax, Constants.HoodPIDSpeedMax);

	}

	/**
	 * Sets the flywheel speed. The shooter will turn off is the speed is set to 0
	 * and will automatically turn on if it is greater than 0.
	 * 
	 * Will do nothing is the robot is currently homeing.
	 * 
	 * @param speed Speed to set flywheel too
	 */
	public synchronized void setSpeed(double speed) {
		targetShooterSpeed = speed;
		if (shooterState == ShooterState.HOMING)
			return;

		if (speed > 0) {
			shooterState = ShooterState.SPINNING;
		} else {
			shooterState = ShooterState.OFF;
		}
	}

	/**
	 * Note the shooter will only fire if the hood has reach the correct position
	 * and if the shooter is inside the max deviation
	 * 
	 * @param fire sets if the shooter should fire balls
	 */
	public synchronized void setFiring(boolean fire) {
		firing = fire;

	}

	public synchronized void update() {
		// System.out.println("Speed: " + shooterOutput + " Error: " + flywheelError );

		switch (shooterState) {
			case SPINNING:
				shooterMaster.set(ControlMode.Velocity, targetShooterSpeed / Constants.ShooterRPMPerTicksPer100ms);
				hoodPID.setReference(targetHoodPosition, ControlType.kPosition);

				// check if motor has sped up
				if (Math.abs(flywheelError) < Constants.ShooterMaxDeviation) {
					double hoodError = targetHoodPosition - hoodEncoder.getPosition();

					if (Math.abs(hoodError) < Constants.HoodMaxDeviation && firing) { // TODO: make hood do things
						// Hood Ready
						feederMotor.set(ControlMode.PercentOutput, Constants.FeederMotorSpeed);
					} else {
						feederMotor.set(ControlMode.PercentOutput, 0);
					}

				}
				break;

			case OFF:
				shooterMaster.set(ControlMode.PercentOutput, 0);
				feederMotor.set(ControlMode.PercentOutput, 0);
				hoodPID.setReference((1) * Constants.HoodRotationsPerDegree, ControlType.kPosition);
				break;

			case HOMING:
				// System.out.println("trying to home");
				if (timerStarted == false) {
					timerStarted = true;
					hoodHomeStart = Timer.getFPGATimestamp();

				}
				feederMotor.set(ControlMode.PercentOutput, 0);
				shooterMaster.set(ControlMode.PercentOutput, 0);

				hoodMotor.set(Constants.HoodHomingSpeed);
				if (getHomeSwitch()) {
					hoodEncoder.setPosition(0);
					shooterState = ShooterState.OFF;
					hoodPID.setReference(0, ControlType.kPosition);
					targetHoodPosition = 0;
					System.out.println("Homing succeeded!");

				} else if (Timer.getFPGATimestamp() > hoodHomeStart + Constants.HoodHomeTimeout) {
					hoodEncoder.setPosition(0);
					shooterState = ShooterState.OFF;
					hoodPID.setReference(0, ControlType.kPosition);
					targetHoodPosition = 0;
					System.out.println("Home Failed");

				}

				break;
			case EJECT:
				feederMotor.set(ControlMode.PercentOutput, -Constants.FeederMotorSpeed);
				shooterMaster.set(ControlMode.PercentOutput, 0);

		}

	}

	/**
	 * Will check if the hood angle is within acceptable ranges before setting the
	 * position
	 * 
	 * @param angle sets the hood angle starting from the closed position
	 */
	public synchronized void setHoodAngle(double angle) {
		if (angle < Constants.MinHoodReleaseAngle) {
			angle = Constants.MinHoodReleaseAngle;
		}

		if (angle > Constants.MaxHoodReleaseAngle) {
			angle = Constants.MaxHoodReleaseAngle;
		}
		targetHoodPosition = (Constants.MaxHoodReleaseAngle - angle) * Constants.HoodRotationsPerDegree;
	}

	/**
	 * 
	 * @return the current hood angle
	 */
	public double getHoodAngle() {
		return -(hoodEncoder.getPosition() / Constants.HoodRotationsPerDegree) + Constants.MaxHoodReleaseAngle;
	}

	/**
	 * Starts the homeing routine for the hood. Fallback timer will begin one the
	 * robot is enabled.
	 */
	public synchronized void homeHood() {
		timerStarted = false;
		shooterState = ShooterState.HOMING;
		// hoodEncoder.setPosition(0);
	}

	/**
	 * 
	 * @return has the hood finished homeing
	 */
	public synchronized boolean isHomed() {
		return (shooterState != ShooterState.HOMING);
	}

	/**
	 * 
	 * @param eject set to true if the shooter should run in eject mode
	 */
	public synchronized void setEject(boolean eject) {
		if (eject) {
			shooterState = ShooterState.EJECT;
		} else {
			shooterState = ShooterState.OFF;
		}
	}

	/**
	 * 
	 * @return returns true one the shooter has reached an acceptable speed
	 */
	public synchronized boolean isShooterSpeedOKAuto() {
		return Math.abs(getRPM() - targetShooterSpeed) < Constants.AutoShooterAccptableRange;
	}

	@Override
	public void selfTest() {
		
	}

	@Override
	public void logData() {

	}
	


	private double getRPM(){
		return shooterMaster.getSelectedSensorVelocity() * Constants.ShooterRPMPerTicksPer100ms;


	}

	public boolean getHomeSwitch(){
		return !homeSwitch.get();
	}
}