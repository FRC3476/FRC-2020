// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.NavXMPX_Gyro;
import frc.utility.OrangeUtility;
import frc.utility.control.RateLimiter;
import frc.utility.control.SynchronousPid;
import frc.utility.control.motion.Path;
import frc.utility.control.motion.PurePursuitController;
import frc.utility.math.RigidTransform2D;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.utility.LazyCANSparkMax;
import frc.utility.Limelight;

public class Drive extends Subsystem {

	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, HOLD, DONE, RAMSETE
	}

	public static class DriveSignal {
		/*
		 * Inches per second for speed
		 */
		public double leftVelocity;
		public double rightVelocity;
		public double leftAcc;
		public double rightAcc;

		public DriveSignal(double left, double right) {
			this(left, 0, right, 0);
		}

		public DriveSignal(double left, double leftAcc, double right, double rightAcc) {
			leftVelocity = left;
			this.leftAcc = leftAcc;
			rightVelocity = right;
			this.rightAcc = rightAcc;
		}
	}

	public static class AutoDriveSignal {
		public DriveSignal command;
		public boolean isDone;

		public AutoDriveSignal(DriveSignal command, boolean isDone) {
			this.command = command;
			this.isDone = isDone;
		}
	}

	private static final Drive instance = new Drive();

	public static Drive getInstance() {
		return instance;
	}

	private double quickStopAccumulator;

	private boolean drivePercentVbus;

	private NavXMPX_Gyro gyroSensor = new NavXMPX_Gyro(SPI.Port.kMXP);// = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	// private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon,
	// rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private SynchronousPid turnPID;
	private SynchronousPid turnPIDAuto;
	public DriveState driveState;
	private RateLimiter moveProfiler;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;
	boolean rotateAuto = false;

	// TODO: Change
	public DifferentialDriveKinematics ramseteDiffDriveKinematics = Constants.RamseteDiffDriveKinematics;
	RamseteController ramseteController = new RamseteController(4, 0.8);

	double prevPositionL = 0;
	double prevPositionR = 0;

	public boolean isAiming = false;

	double prevTime;

	public LazyCANSparkMax leftSpark, rightSpark, leftSparkSlave, rightSparkSlave, leftSparkSlave2, rightSparkSlave2;
	private CANPIDController leftSparkPID, rightSparkPID;
	private CANEncoder leftSparkEncoder, rightSparkEncoder;

	public LazyCANSparkMax leftFrontSpark, leftBackSpark, rightFrontSpark, rightBackSpark;
	private CANEncoder leftFrontSparkEncoder, leftBackSparkEncoder, rightFrontSparkEncoder, rightBackSparkEncoder;

	public LazyCANSparkMax leftFrontSparkSwerve, leftBackSparkSwerve, rightFrontSparkSwerve, rightBackSparkSwerve;
	private CANEncoder leftFrontSparkEncoderSwerve, leftBackSparkEncoderSwerve, rightFrontSparkEncoderSwerve,
			rightBackSparkEncoderSwerve;
	SwerveDriveKinematics swerveKinematics;
	Trajectory currentAutoTrajectory;

	private Drive() {
		super(Constants.DrivePeriod);
		Pose2d tollerance = new Pose2d(new Translation2D(20,20).getScaledWPITranslation2d(), Rotation2d.fromDegrees(90));
		System.out.println(tollerance);
		ramseteController.setTolerance(tollerance);

		leftSpark = new LazyCANSparkMax(Constants.DriveLeftMasterId, MotorType.kBrushless);
		leftSparkSlave = new LazyCANSparkMax(Constants.DriveLeftSlave1Id, MotorType.kBrushless);
		rightSpark = new LazyCANSparkMax(Constants.DriveRightMasterId, MotorType.kBrushless);
		rightSparkSlave = new LazyCANSparkMax(Constants.DriveRightSlave1Id, MotorType.kBrushless);
		// rightSparkSlave2 = new CANSparkMax(0, MotorType.kBrushless);
		// leftSparkSlave2 = new CANSparkMax(0, MotorType.kBrushless);
		leftSpark.setInverted(true);
		rightSpark.setInverted(false);
		leftSparkSlave.setInverted(true);
		rightSparkSlave.setInverted(false);

		leftSparkPID = leftSpark.getPIDController();
		rightSparkPID = rightSpark.getPIDController();
		leftSparkEncoder = leftSpark.getEncoder();
		rightSparkEncoder = rightSpark.getEncoder();

		// Swerve Drive Motors
		leftFrontSpark = new LazyCANSparkMax(Constants.DriveLeftFrontId, MotorType.kBrushless);
		leftBackSpark = new LazyCANSparkMax(Constants.DriveLeftBackId, MotorType.kBrushless);
		rightFrontSpark = new LazyCANSparkMax(Constants.DriveRightFrontId, MotorType.kBrushless);
		rightBackSpark = new LazyCANSparkMax(Constants.DriveRightBackId, MotorType.kBrushless);

		leftFrontSparkEncoder = leftFrontSpark.getEncoder();
		leftBackSparkEncoder = leftBackSpark.getEncoder();
		rightFrontSparkEncoder = rightFrontSpark.getEncoder();
		rightBackSparkEncoder = rightBackSpark.getEncoder();

		leftFrontSparkSwerve = new LazyCANSparkMax(Constants.DriveLeftFrontSwerveId, MotorType.kBrushless);
		leftBackSparkSwerve = new LazyCANSparkMax(Constants.DriveLeftBackSwerveId, MotorType.kBrushless);
		rightFrontSparkSwerve = new LazyCANSparkMax(Constants.DriveRightFrontSwerveId, MotorType.kBrushless);
		rightBackSparkSwerve = new LazyCANSparkMax(Constants.DriveRightBackSwerveId, MotorType.kBrushless);

		leftFrontSparkEncoderSwerve = leftFrontSparkSwerve.getEncoder();
		leftBackSparkEncoderSwerve = leftBackSparkSwerve.getEncoder();
		rightFrontSparkEncoderSwerve = rightFrontSparkSwerve.getEncoder();
		rightBackSparkEncoderSwerve = rightBackSparkSwerve.getEncoder();

		// Creating kinematics object using the module locations
		swerveKinematics = new SwerveDriveKinematics(Constants.LeftFrontLocation, Constants.LeftBackLocation,
				Constants.RightFrontLocation, Constants.RightBackLocation);

		// leftSparkPID.

		// rightSparkSlave.follow(rightSpark);
		// leftSparkSlave.follow(leftSpark);
		// leftSparkSlave2.follow(leftSpark);
		// rightSparkSlave2.follow(rightSpark);
		configMotors();

		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		turnPID = new SynchronousPid(2, 0, 16, 0); // P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DriveHighSpeed / 5, -Constants.DriveHighSpeed / 5);
		turnPID.setSetpoint(0);
		turnPIDAuto = new SynchronousPid(2, 0, 16, 0); // P=1.0 OR 0.8
		turnPIDAuto.setOutputRange(Constants.DriveHighSpeed / 8, -Constants.DriveHighSpeed / 8);
		turnPIDAuto.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.DriveTeleopAccLimit);

		configHigh();
		configAuto();
	}

	public void debug() {
		System.out.println("L enc: " + getLeftDistance() + " velo " + getLeftSpeed());
		System.out.println("R enc: " + getRightDistance() + " velo " + getRightSpeed());
		System.out.println("Gyro: " + getAngle()/* getGyroAngle().getDegrees() */);
	}

	public void debugSpeed() {
		System.out.println("L speed " + " actual " + getLeftSpeed());
		System.out.println("R speed " + " actual " + getRightSpeed());

	}

	public void setRight() {
		setWheelVelocity(new DriveSignal(40, 0));
	}

	public void configBrake() {
		configMotorsCoastBrake(IdleMode.kBrake);
	}

	public void configCoast(){
		configMotorsCoastBrake(IdleMode.kCoast);
	}

	IdleMode setIdleMode = IdleMode.kBrake;
	boolean fixIdleMode = false;
	int delay;

	public void configMotorsCoastBrake(IdleMode mode) {
		boolean error = false;
		if (CANError.kOk != leftSpark.setIdleMode(mode) || error)
			error = true;
		if (CANError.kOk != rightSpark.setIdleMode(mode) || error)
			error = true;
		if (CANError.kOk != leftSparkSlave.setIdleMode(mode) || error)
			error = true;
		if (CANError.kOk != rightSparkSlave.setIdleMode(mode) || error)
			error = true;
		if(error){
			System.out.println("failed to enable " + mode.toString() + " mode");
			setIdleMode = mode;
			fixIdleMode = true;
			delay = 4;
		}
		



	}

	private void configAuto() {
		/*
		 * rightTalon.config_kP(0, Constants.kDriveRightAutoP, 10);
		 * rightTalon.config_kD(0, Constants.kDriveRightAutoD, 10);
		 * rightTalon.config_kF(0, Constants.kDriveRightAutoF, 10);
		 * leftTalon.config_kP(0, Constants.kDriveLeftAutoP, 10); leftTalon.config_kD(0,
		 * Constants.kDriveRightAutoD, 10); leftTalon.config_kF(0,
		 * Constants.kDriveLeftAutoF, 10); driveMultiplier = Constants.DriveHighSpeed;
		 * rightTalon.configClosedloopRamp(12d / 200d, 10);
		 * leftTalon.configClosedloopRamp(12d / 200d, 10);
		 */
		// System.out.println(rightTalon.)
		rightSparkPID.setP(Constants.kDriveRightAutoP, 0);
		rightSparkPID.setD(Constants.kDriveRightAutoD, 0);
		rightSparkPID.setFF(Constants.kDriveRightAutoF, 0);
		rightSparkPID.setOutputRange(-1, 1);

		leftSparkPID.setP(Constants.kDriveLeftAutoP, 0);
		leftSparkPID.setD(Constants.kDriveLeftAutoD, 0);
		leftSparkPID.setFF(Constants.kDriveLeftAutoF, 0);
		leftSparkPID.setOutputRange(-1, 1);

	}

	private void configHigh() {
		// rightTalon.config_kP(0, Constants.kDriveRightHighP, 10);
		// rightTalon.config_kD(0, Constants.kDriveRightHighD, 10);
		// rightTalon.config_kF(0, Constants.kDriveRightHighF, 10);
		// rightTalon.configClosedloopRamp(12d / 200d, 10);
		// leftTalon.config_kP(0, Constants.kDriveLeftHighP, 10);
		// leftTalon.config_kD(0, Constants.kDriveRightHighD, 10);
		// leftTalon.config_kF(0, Constants.kDriveLeftHighF, 10);
		// leftTalon.configClosedloopRamp(12d / 200d, 10);

		driveMultiplier = Constants.DriveHighSpeed;
	}

	boolean teleopstart = true;

	private double autoStartTime;

	synchronized public void setTeleop() {
		driveState = DriveState.TELEOP;
	}

	public void arcadeDrive(double moveValue, double rotateValue) {
		// String toPrint="";
		// double time = Timer.getFPGATimestamp();
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, Constants.steeringWheel ? 1 : 0);
		// double t = Timer.getFPGATimestamp() - time;
		// if(teleopstart) toPrint += (t) + " 12\n";

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);
		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}
		// if(teleopstart) toPrint += (Timer.getFPGATimestamp() - time) + " 12\n";

		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		if (drivePercentVbus) {
			// System.out.println("left " + getLeftSpeed() + " power: " + leftMotorSpeed);
			// System.out.println("right " + getRightSpeed() + " power: " +
			// rightMotorSpeed);

			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed = moveValue + rotateValue * 0.5;
			rightMotorSpeed = moveValue - rotateValue * 0.5;
			leftMotorSpeed *= Constants.DriveHighSpeed;
			rightMotorSpeed *= Constants.DriveHighSpeed;

			// System.out.println("left " + (leftMotorSpeed - getLeftSpeed() ));
			// System.out.println("right " + (rightMotorSpeed - getRightSpeed() ));
			// System.out.println("left " + leftMotorSpeed + " right " + rightMotorSpeed);
			// System.out.println("REQUESTED WHELEVE VROLOCITY: " + leftMotorSpeed + ", " +
			// rightMotorSpeed);

			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
		// System.out.println("left motor speed " + leftMotorSpeed + " right motor speed
		// " + rightMotorSpeed);
		// if(teleopstart) toPrint += (Timer.getFPGATimestamp() - time) + " 12\n";
		// System.out.print(toPrint);
		teleopstart = false;
		// System.out.println("Left: " + moveValue + ", " +
		// leftSparkEncoder.getVelocity());
		// System.out.println("Right: " + moveValue + ", " +
		// rightSparkEncoder.getVelocity());
	}

	public void calibrateGyro() {
		gyroSensor.calibrate();
	}

	public void printCurrent() {
		System.out.println(leftSpark);
	}

	public void startHold() {
		prevPositionL = getLeftDistance();
		prevPositionR = getRightDistance();
		driveState = DriveState.HOLD;
		configHigh();
	}

	public void endHold() {
		driveState = DriveState.TELEOP;
	}

	public void cheesyDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, Constants.steeringWheel ? 1 : 0);

		double leftMotorSpeed;
		double rightMotorSpeed;
		double angularPower = 1;

		double overPower;

		if (isQuickTurn) {
			overPower = 1;
			if (moveValue < 0.2) {
				quickStopAccumulator = 0.9 * quickStopAccumulator + 0.1 * rotateValue * 2;
			}
			angularPower = rotateValue * 0.4; // 0.2
		} else {
			overPower = 0;
			angularPower = 1.1 * Math.abs(moveValue) * rotateValue - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		// moveValue = moveProfiler.update(moveValue * driveMultiplier) /
		// driveMultiplier;
		leftMotorSpeed = moveValue + angularPower;
		rightMotorSpeed = moveValue - angularPower;

		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= overPower * (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= overPower * (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += overPower * (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += overPower * (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}

		leftMotorSpeed = OrangeUtility.coerce(leftMotorSpeed, 1, -1);
		rightMotorSpeed = OrangeUtility.coerce(rightMotorSpeed, 1, -1);

		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void hold() {
		// leftSparkPID.setReference(leftSparkEncoder.getPosition(),
		// ControlType.kPosition);
		// rightSparkPID.setReference(leftSparkEncoder.getPosition(),
		// ControlType.kPosition);
		// driveState = DriveState.HOLD;
		double errorL = prevPositionL - getLeftDistance();
		double errorR = prevPositionR - getRightDistance();
		setWheelVelocity(new DriveSignal(errorL * Constants.kHoldP, errorR * Constants.kHoldP));

	}

	public void orangeDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, Constants.steeringWheel ? 1 : 0);
		// 50 is min turn radius
		double radius = (1 / rotateValue) + Math.copySign(24, rotateValue);
		double deltaSpeed = (Constants.TrackRadius * ((moveValue * driveMultiplier) / radius));
		deltaSpeed /= driveMultiplier;
		if (isQuickTurn) {
			deltaSpeed = rotateValue;
		}
		double leftMotorSpeed = moveValue + deltaSpeed;
		double rightMotorSpeed = moveValue - deltaSpeed;
		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}
		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			if (leftMotorSpeed == 0 && rightMotorSpeed == 0) {
				setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
			}
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void skidLimitingDrive(double moveValue, double rotateValue) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, 0);

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);

		// Linear
		/*
		 * double slope = -1.25; double maxRotate = slope * Math.abs(moveValue) + 1;
		 */
		// Nonlinear
		double curvature = 4;
		double curveCenter = 0.5;

		// Concave up
		// y = (0.5 / (5 * x)) - (0.5 / 5)
		// double maxRotate = curveCenter / (curvature * Math.abs(moveValue)) -
		// (curveCenter / curvature);

		// Concave down
		// y = -2^(5 * (x - 1)) + 1
		double maxRotate = -Math.pow((1 / curveCenter), curvature * (Math.abs(moveValue) - 1)) + 0.8;

		rotateValue = OrangeUtility.coerce(rotateValue, maxRotate, -maxRotate);

		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}

		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		if (drivePercentVbus) {

			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed = moveValue + rotateValue;
			rightMotorSpeed = moveValue - rotateValue;
			leftMotorSpeed *= Constants.DriveHighSpeed;
			rightMotorSpeed *= Constants.DriveHighSpeed;
			// System.out.println(leftMotorSpeed +" , " + rightMotorSpeed);
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void swerveDrive(double x1, double x2, double y1) {

		swerveDrive(
			new ChassisSpeeds((Constants.DriveHighSpeed / 100) * x1, (Constants.DriveHighSpeed / 100) * x2, y1));

	}

	public void swerveDriveFieldRelitive(double x1, double x2, double y1){

		swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds((Constants.DriveHighSpeed / 100) * x1,
			(Constants.DriveHighSpeed / 100) * x2, y1, Rotation2d.fromDegrees(getAngle())));

	}

	private void swerveDrive(ChassisSpeeds chassisSpeeds) {

		/*
		 * Things to cahnge before using 1. Ids 2. set Locations of all wheels 3.
		 */

		// TODO: Set motor control modes

		synchronized (this) {
			driveState = DriveState.TELEOP;
		}

		SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DriveHighSpeed / 100);

		SwerveModuleState leftFront = moduleStates[0];
		SwerveModuleState leftBack = moduleStates[1];
		SwerveModuleState rightFront = moduleStates[2];
		SwerveModuleState rightBack = moduleStates[3];

		double leftFrontSpeed = leftFront.speedMetersPerSecond * 100;
		double leftBackSpeed = leftBack.speedMetersPerSecond * 100;
		double rightFrontSpeed = rightFront.speedMetersPerSecond * 100;
		double rightBackSpeed = rightBack.speedMetersPerSecond * 100;

		leftFrontSpark.set(leftFrontSpeed);
		leftBackSpark.set(leftBackSpeed);
		rightFrontSpark.set(rightFrontSpeed);
		rightBackSpark.set(rightBackSpeed);

		leftFrontSparkSwerve.set(leftFront.angle.getDegrees());
		leftBackSparkSwerve.set(leftBack.angle.getDegrees());
		rightFrontSparkSwerve.set(rightFront.angle.getDegrees());
		rightBackSparkSwerve.set(rightBack.angle.getDegrees());
	}

	private void configMotors() {
		leftSparkSlave.follow(leftSpark);
		rightSparkSlave.follow(rightSpark);

		configBrake();

		// leftSparkEncoder.setInverted(true);
		// rightSparkEncoder.setInverted(true);
		/*
		 * leftSlaveTalon.set(ControlMode.Follower, Constants.DriveLeftMasterId);
		 * leftSlave2Talon.set(ControlMode.Follower, Constants.DriveLeftMasterId);
		 * rightSlaveTalon.set(ControlMode.Follower, Constants.DriveRightMasterId);
		 * rightSlave2Talon.set(ControlMode.Follower, Constants.DriveRightMasterId);
		 * setBrakeState(NeutralMode.Brake);
		 * 
		 * leftTalon.setInverted(true); leftSlaveTalon.setInverted(true);
		 * leftSlave2Talon.setInverted(true);
		 * 
		 * rightTalon.setInverted(false); rightSlaveTalon.setInverted(false);
		 * rightSlave2Talon.setInverted(false);
		 * 
		 * leftTalon.setSensorPhase(false); rightTalon.setSensorPhase(false);
		 * 
		 * rightTalon.setNeutralMode(NeutralMode.Brake);
		 * leftTalon.setNeutralMode(NeutralMode.Brake);
		 * rightSlaveTalon.setNeutralMode(NeutralMode.Brake);
		 * leftSlaveTalon.setNeutralMode(NeutralMode.Brake);
		 * rightSlave2Talon.setNeutralMode(NeutralMode.Brake);
		 * leftSlave2Talon.setNeutralMode(NeutralMode.Brake);
		 */

	}

	public void resetMotionProfile() {
		moveProfiler.reset();
	}

	public double getAngle() {
		return gyroSensor.getAngle();
	}

	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	public Rotation2D getGyroAngle() {
		// -180 through 180
		return Rotation2D.fromDegrees(gyroSensor.getAngle());
	}

	public double getLeftDistance() {
		/*
		 * return leftTalon.getSelectedSensorPosition(0) /
		 * Constants.EncoderTicksPerRotation * Constants.WheelDiameter Math.PI * 22d /
		 * 62d / 3d;
		 */
		return leftSparkEncoder.getPosition() * Constants.kDriveInchesPerRevolution;
	}

	public double getRightDistance() {
		// return rightTalon.getSelectedSensorPosition(0) /
		// Constants.EncoderTicksPerRotation * Constants.WheelDiameter
		// * Math.PI * 22d / 62d / 3d;
		return rightSparkEncoder.getPosition() * Constants.kDriveInchesPerRevolution;
	}

	public double getSpeed() {
		/*
		 * return (-leftSparkEncoder.getVelocity() + rightSparkEncoder.getVelocity()) /
		 * 10 / 2 * Constants.WheelDiameter * Math.PI;
		 */
		return (getLeftSpeed() + getRightSpeed()) / 2;
	}

	public double getLeftSpeed() {
		return leftSparkEncoder.getVelocity() * Constants.kDriveInchesPerSecPerRPM;
	}

	public double getRightSpeed() {
		return rightSparkEncoder.getVelocity() * Constants.kDriveInchesPerSecPerRPM;
	}

	public double scaleJoystickValues(double rawValue, int profile) {
		return Math.copySign(
				OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinControllerInput[profile],
						Constants.MaxControllerInput, Constants.MinControllerOutput, Constants.MaxControllerOutput),
				rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.PUREPURSUIT;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		autonomousDriver.resetTime();
		configAuto();
		updatePurePursuit();
	}

	public synchronized void setAutoPath(Trajectory trajectory) {
		autoStartTime = Timer.getFPGATimestamp();
		driveState = DriveState.RAMSETE;
		this.currentAutoTrajectory = trajectory;
		autoStartTime = Timer.getFPGATimestamp();
		configAuto();
		configCoast();
		updateRamsete();
	}

	public void setBrakeState(NeutralMode mode) {
		// leftTalon.setNeutralMode(mode);
		// rightTalon.setNeutralMode(mode);
		// leftSlaveTalon.setNeutralMode(mode);
		// rightSlaveTalon.setNeutralMode(mode);
		// leftSlave2Talon.setNeutralMode(mode);
		// rightSlave2Talon.setNeutralMode(mode);
	}

	public double getVoltage() {
		return 0;
		// return (leftTalon.getMotorOutputVoltage() +
		// rightTalon.getMotorOutputVoltage()
		// + leftSlaveTalon.getMotorOutputVoltage() +
		// rightSlaveTalon.getMotorOutputVoltage()
		// + rightSlave2Talon.getMotorOutputVoltage() +
		// leftSlave2Talon.getMotorOutputVoltage()) / 6;
	}

	private void setWheelPower(DriveSignal setVelocity) {
		// leftTalon.set(ControlMode.PercentOutput, setVelocity.rightVelocity);
		// rightTalon.set(ControlMode.PercentOutput, setVelocity.leftVelocity);
		// System.out.println(leftSpark.getLastError());

		// System.out.println(setVelocity.leftVelocity + ", " +
		// setVelocity.rightVelocity);

		leftSpark.set(setVelocity.leftVelocity);
		leftSparkSlave.set(setVelocity.leftVelocity); // tmp //TODO

		rightSpark.set(setVelocity.rightVelocity);
		rightSparkSlave.set(setVelocity.rightVelocity); // tmp

		/*
		 * System.out.println( "Left Spark: " + leftSpark.getOutputCurrent() + "\n" +
		 * "Left Slave: " + leftSparkSlave.getOutputCurrent() + "\n" + "Right Spark: " +
		 * rightSpark.getOutputCurrent() + "\n" + "Right Slave: " +
		 * rightSparkSlave.getOutputCurrent() + "\n" );
		 */
		// System.out.println("velo: " + setVelocity.leftVelocity);

		// leftSparkPID.setReference(setVelocity.leftVelocity, ControlType.kDutyCycle);
		// rightSparkPID.setReference(setVelocity.rightVelocity,
		// ControlType.kDutyCycle);
	}

	public boolean hasStickyFaults() {
		short kCANRXmask = (short) (1 << CANSparkMax.FaultID.kCANRX.ordinal());
		short kCANTXmask = (short) (1 << CANSparkMax.FaultID.kCANTX.ordinal());
		short kHasResetmask = (short) (1 << CANSparkMax.FaultID.kHasReset.ordinal());
		short leftFaults = leftSpark.getStickyFaults();
		short rightFaults = rightSpark.getStickyFaults();
		// System.out.println("left: " + leftFaults + " right: " + rightFaults);

		boolean leftSticky = (leftFaults & kCANRXmask) != 0 || (leftFaults & kCANTXmask) != 0
				|| (leftFaults & kHasResetmask) != 0;
		boolean rightSticky = (rightFaults & kCANRXmask) != 0 || (rightFaults & kCANTXmask) != 0
				|| (rightFaults & kHasResetmask) != 0;
		return leftSticky || rightSticky;
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > Constants.DriveHighSpeed
				|| Math.abs(setVelocity.leftVelocity) > Constants.DriveHighSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.DriveHighSpeed + " !", false);
			return;
		}
		// System.out.println("Left: " + setVelocity.leftVelocity);
		// + getLeftSpeed());
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.leftVelocity) / Constants.kDriveInchesPerSecPerRPM;
		double rightSetpoint = (setVelocity.rightVelocity) / Constants.kDriveInchesPerSecPerRPM;

		// leftTalon.set(ControlMode.Velocity, leftSetpoint);
		// rightTalon.set(ControlMode.Velocity, rightSetpoint);


		//leftTalon.set(ControlMode.Velocity, leftSetpoint);
		//rightTalon.set(ControlMode.Velocity, rightSetpoint);

		//Open Loop
		leftSpark.set(setVelocity.leftVelocity/Constants.DriveHighSpeed);
		rightSpark.set(setVelocity.rightVelocity/Constants.DriveHighSpeed);
		
		//Closed Loop
		// leftSparkPID.setReference(leftSetpoint, ControlType.kVelocity);
		// rightSparkPID.setReference(rightSetpoint, ControlType.kVelocity);

		//System.out.println("desired left rpm: " + setVelocity.leftVelocity + " desired right rpm: " + setVelocity.rightVelocity);
		// System.out.println("actual left rpm: " + getLeftSpeed() + " actual right rpm:
		// " + getRightSpeed());

	}

	public synchronized void setSimpleDrive(boolean setting) {
		if (drivePercentVbus != setting)
			System.out.println("Simple drive: " + setting);
		drivePercentVbus = setting;
	}

	public synchronized boolean getSimpleDrive() {
		return drivePercentVbus;
	}

	@Override
	public void update() {
		// System.out.println("L speed " + getLeftSpeed() + " position x " +
		// RobotTracker.getInstance().getOdometry().translationMat.getX());
		// System.out.println("R speed " + getRightSpeed() + " position y " +
		// RobotTracker.getInstance().getOdometry().translationMat.getY());
		// debugSpeed();
		// System.out.println(driveState);
		DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
			case TELEOP:
				// System.out.println(leftSpark.getStickyFaults() + " slave faults " +
				// leftSparkSlave.getStickyFaults());
				// CANSparkMax.FaultID.kCANTX

				break;
			case PUREPURSUIT:
				//System.out.println("bad!");
				updatePurePursuit();
				break;
			case TURN:
				//System.out.println("turning");
				updateTurn();
				break;
			case RAMSETE:
				updateRamsete();
				break;
			case HOLD:
				hold();
				//System.out.println("holding");
				break;
		}

		//CAUSES CONSTANT THREAD SLEEP FAIL
		// boolean error = false;
		// if (CANError.kOk != leftSpark.setIdleMode(setIdleMode) || error)
		// 	error = true;
		// if (CANError.kOk != rightSpark.setIdleMode(setIdleMode) || error)
		// 	error = true;
		// if (CANError.kOk != leftSparkSlave.setIdleMode(setIdleMode) || error)
		// 	error = true;
		// if (CANError.kOk != rightSparkSlave.setIdleMode(setIdleMode) || error)
		// 	error = true;
		// if(error){
		// 	System.out.println("failed to enable " + setIdleMode.toString() + " mode");
		// }
		
		
	}

	private void updateRamsete() {
		Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp()-autoStartTime);
		//System.out.println(goal);
		RigidTransform2D transform = RobotTracker.getInstance().getOdometry();
		Pose2d currentRobotPose = new Pose2d(transform.translationMat.getScaledWPITranslation2d(),
		transform.rotationMat.getWPIRotation2d());
		ChassisSpeeds adjustedSpeeds = ramseteController.calculate(currentRobotPose, goal);
		DifferentialDriveWheelSpeeds wheelspeeds = ramseteDiffDriveKinematics.toWheelSpeeds(adjustedSpeeds);
		setWheelVelocity(new DriveSignal(Units.metersToInches(wheelspeeds.leftMetersPerSecond), 
			Units.metersToInches(wheelspeeds.rightMetersPerSecond)));
		//System.out.println(ramseteController.atReference());
		//System.out.println("target speed" + Units.metersToInches(wheelspeeds.leftMetersPerSecond) + " " + Units.metersToInches(wheelspeeds.rightMetersPerSecond) + "time: " +(Timer.getFPGATimestamp()-autoStartTime) );
		System.out.println("Goal: (" + goal.poseMeters.getTranslation().getX() + ", " + goal.poseMeters.getTranslation().getY() + ") Actual: (" + currentRobotPose.getX() + ", " + currentRobotPose.getY() + ")");
		//TODO: not working
		if((Timer.getFPGATimestamp()-autoStartTime)>= currentAutoTrajectory.getTotalTimeSeconds()){
		}
		if(ramseteController.atReference() && (Timer.getFPGATimestamp()-autoStartTime)>= currentAutoTrajectory.getTotalTimeSeconds()){
			driveState = DriveState.DONE;
			stopMovement();
		}
	}

	public double getRamseteCompletePercent(){
		return (Timer.getFPGATimestamp()-autoStartTime)/ currentAutoTrajectory.getTotalTimeSeconds();
	}
	synchronized public boolean isAiming() {
		return isAiming; 
	}

	public void setRotation(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
			if(driveState != DriveState.TURN){
				turnMinSpeed = 3;
			}
			driveState = DriveState.TURN;
			rotateAuto = true;
			isAiming = !getTurningDone();
			configBrake();
			
		}
		configHigh();
	}

	public void setRotationTeleop(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
			rotateAuto = false;
			isAiming = !getTurningDone();
			configBrake();
			
		}
		configHigh();
	}

	public synchronized boolean getTurningDone(){
		double error = wantedHeading.inverse().rotateBy(RobotTracker.getInstance().getOdometry().rotationMat).getDegrees();
		return (Math.abs(error) < Constants.maxTurnError && Math.abs(getLeftSpeed()-getRightSpeed()) < Constants.maxPIDStopSpeed);

	}

	double turnMinSpeed;

	private void updateTurn() {
		double error = wantedHeading.rotateBy(RobotTracker.getInstance().getOdometry().rotationMat).getDegrees();
		double curSpeed =  Math.abs(getLeftSpeed()-getRightSpeed());
		double deltaSpeed;
		double pidDeltaSpeed;

		if (rotateAuto) {
			pidDeltaSpeed = turnPIDAuto.update(error);
			deltaSpeed = Math.copySign(Math.max(Math.abs(pidDeltaSpeed), 3), pidDeltaSpeed);
		} else {
			pidDeltaSpeed = turnPID.update(error);
			deltaSpeed = Math.copySign(Math.max(Math.abs(pidDeltaSpeed), turnMinSpeed), pidDeltaSpeed);
		}
		
		if (((Limelight.getInstance().getDistance() >= 160 && Math.abs(error) < Constants.maxTurnErrorFar) || 
				(Limelight.getInstance().getDistance() < 160 && Math.abs(error) < Constants.maxTurnError)) && curSpeed < Constants.maxPIDStopSpeed) {
			setWheelVelocity(new DriveSignal(0, 0));
			isAiming = false;
			
			if( rotateAuto ) {
				synchronized (this) {
					configBrake();
					driveState = DriveState.DONE;
				}
			}
			
		} else {
			System.out.println("Error: " + error + " curSpeed: " + curSpeed + " command: " + deltaSpeed + " pidOut: " 
				+ pidDeltaSpeed + " minSpeed: " + turnMinSpeed);
			isAiming = true;
			setWheelVelocity(new DriveSignal(-deltaSpeed, deltaSpeed));

			if(curSpeed < 0.5) {
				//Updates every 10ms
				turnMinSpeed = Math.min(turnMinSpeed + 0.1, 6);
			} else {
				turnMinSpeed = 2;
			}
		}
	}


	private void updatePurePursuit() {
	//	System.out.println("updating pure presuit");
		AutoDriveSignal signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		if (signal.isDone) {
			setWheelVelocity(signal.command);
			synchronized (this) {
				driveState = DriveState.DONE;
			}
			configHigh();
		}
		//System.out.println("signal l:" + signal.command.leftVelocity + " signal R " + signal.command.rightVelocity);
		setWheelVelocity(signal.command);
	}

	public void resetGyro() {
		gyroSensor.reset();
	}

	public boolean checkSubsystem() {

		// TODO: Get accurate thresholds
		// TODO: Use PDP to get current
		// boolean success =
		//boolean success = leftTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0;
		//success = rightTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0 && success;
		//success = OrangeUtility.checkMotors(.25, Constants.DriveExpectedCurrent, Constants.DriveExpectedRPM,
		//		Constants.DriveExpectedPosition, rightTalon, rightTalon, rightSlaveTalon, rightSlave2Talon);
		//success = OrangeUtility.checkMotors(.25, Constants.DriveExpectedCurrent, Constants.DriveExpectedRPM,
		//		Constants.DriveExpectedPosition, leftTalon, leftTalon, leftSlaveTalon, leftSlave2Talon) && success;
		configMotors();
		return true;
		//return success;
	}

	synchronized public void stopMovement() {
		//leftTalon.set(ControlMode.PercentOutput, 0);
		//rightTalon.set(ControlMode.PercentOutput, 0);
		leftSpark.set(0);
		//leftSpark.set(ControlMode.Current, 3);
		rightSpark.set(0);
		leftSparkPID.setReference(0, ControlType.kDutyCycle);
		rightSparkPID.setReference(0, ControlType.kDutyCycle);
		setWheelVelocity(new DriveSignal(0,0));

		driveState = DriveState.TELEOP;
		//System.out.println("not good");
		resetMotionProfile();
	}

	synchronized public boolean isFinished() {
		return driveState == DriveState.DONE || driveState == DriveState.TELEOP;
	}

	public void clearStickyFaults() {

	}

	@Override
	public void selfTest() {

	}

	@Override
	public void logData() {

	}
}
