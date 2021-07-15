// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.utility.LazyCANSparkMax;
import frc.utility.NavXMPX_Gyro;
import frc.utility.OrangeUtility;
import frc.utility.control.RateLimiter;
import frc.utility.control.SynchronousPid;
import frc.utility.control.motion.Path;
import frc.utility.control.motion.PurePursuitController;
import frc.utility.math.Rotation2D;


public class Drive extends Subsystem {

	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, HOLD, DONE
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

	private NavXMPX_Gyro gyroSensor;// = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	//private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private SynchronousPid turnPID;
	private SynchronousPid turnPIDAuto;
	public DriveState driveState;
	private RateLimiter moveProfiler;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;
	boolean rotateAuto = false; 


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
	private CANEncoder leftFrontSparkEncoderSwerve, leftBackSparkEncoderSwerve, rightFrontSparkEncoderSwerve, rightBackSparkEncoderSwerve;
	SwerveDriveKinematics swerveKinematics;

	public LazyCANSparkMax[] swerveMotors = new LazyCANSparkMax[4];
	public LazyCANSparkMax[] swerveDriveMotors = new LazyCANSparkMax[4]; 
	CANEncoder[] swerveEncoders = new CANEncoder[4];
	  
	double kp = 0.01;
	double kd = 0;
	double kf = 0.000000;

	CANPIDController[] swervePID = new CANPIDController[4];


	private Drive() {
		super(Constants.DrivePeriod);
		gyroSensor = new NavXMPX_Gyro(SPI.Port.kMXP);

		//Swerve Drive Motors
		leftFrontSpark = new LazyCANSparkMax(Constants.DriveLeftFrontId, MotorType.kBrushless);
		leftBackSpark = new LazyCANSparkMax(Constants.DriveLeftBackId, MotorType.kBrushless);
		rightFrontSpark = new LazyCANSparkMax(Constants.DriveRightFrontId, MotorType.kBrushless);
		rightBackSpark = new LazyCANSparkMax(Constants.DriveRightBackId, MotorType.kBrushless);

		leftFrontSparkEncoder = leftFrontSpark.getEncoder();
		leftBackSparkEncoder  = leftBackSpark.getEncoder();
		rightFrontSparkEncoder = rightFrontSpark.getEncoder();
		rightBackSparkEncoder = rightBackSpark.getEncoder();

		leftFrontSparkSwerve = new LazyCANSparkMax(Constants.DriveLeftFrontSwerveId, MotorType.kBrushless);
		leftBackSparkSwerve = new LazyCANSparkMax(Constants.DriveLeftBackSwerveId, MotorType.kBrushless);
		rightFrontSparkSwerve = new LazyCANSparkMax(Constants.DriveRightFrontSwerveId, MotorType.kBrushless);
		rightBackSparkSwerve = new LazyCANSparkMax(Constants.DriveRightBackSwerveId, MotorType.kBrushless);

		

		
		swerveMotors[0] = leftFrontSparkSwerve;
		swerveMotors[1] = leftBackSparkSwerve;
		swerveMotors[2] = rightFrontSparkSwerve;
		swerveMotors[3] = rightBackSparkSwerve;

		swerveDriveMotors[0] = leftFrontSpark;
		swerveDriveMotors[1] = leftBackSpark;
		swerveDriveMotors[2] = rightFrontSpark;
		swerveDriveMotors[3] = rightBackSpark;



		for(int i = 0; i <4; i++){
			swerveEncoders[i] = swerveMotors[i].getEncoder();
			swerveEncoders[i].setPositionConversionFactor(8.1466);
			swerveMotors[i].getAnalog(AnalogMode.kAbsolute).setPositionConversionFactor(105.88);
			angleOffsets[i] = 0; // -swerveMotors[i].getAnalog(AnalogMode.kAbsolute).getPosition();
			swervePID[i] = swerveMotors[i].getPIDController();
			swervePID[i].setP(kp);
			swervePID[i].setD(kd);
			swervePID[i].setI(0);
			swervePID[i].setFF(kf);

			swerveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
			
		
		}

		leftFrontSparkEncoderSwerve = leftFrontSparkSwerve.getEncoder();
		leftBackSparkEncoderSwerve  = leftBackSparkSwerve.getEncoder();
		rightFrontSparkEncoderSwerve = rightFrontSparkSwerve.getEncoder();
		rightBackSparkEncoderSwerve = rightBackSparkSwerve.getEncoder();
		
		

		// Creating kinematics object using the module locations
		swerveKinematics = new SwerveDriveKinematics(Constants.LeftFrontLocation, Constants.LeftBackLocation, Constants.RightFrontLocation,Constants.RightBackLocation);
	

		//leftSparkPID.b
		
		//rightSparkSlave.follow(rightSpark);
		//leftSparkSlave.follow(leftSpark);
		//leftSparkSlave2.follow(leftSpark);
		//rightSparkSlave2.follow(rightSpark);
		configMotors();

		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		turnPID = new SynchronousPid(3.5, 0, 0.0, 0); //P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DriveHighSpeed/5, -Constants.DriveHighSpeed/5);
		turnPID.setSetpoint(0);
		turnPIDAuto = new SynchronousPid(1, 0, 0, 0); //P=1.0 OR 0.8
		turnPIDAuto.setOutputRange(Constants.DriveHighSpeed/8, -Constants.DriveHighSpeed/8);
		turnPIDAuto.setSetpoint(0);
		

		moveProfiler = new RateLimiter(Constants.DriveTeleopAccLimit);

		configHigh();
		configAuto();
	}



	private void configBrake() {
		//TODO
	}

	private void configCoast() {
		//TODO
	}

	private void configAuto() {
		

		
	}

	private void configHigh() {

		driveMultiplier = Constants.DriveHighSpeed;
	}


	boolean teleopstart =true;

	synchronized public void setTeleop() {
		driveState = DriveState.TELEOP;
	}

	synchronized public SwerveModuleState[] getSwerveModuleStates(){
		SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
		for(int i = 0; i<4; i++){
			SwerveModuleState moduleState = new SwerveModuleState(swerveDriveMotors[i].getEncoder().getVelocity()*Constants.SwerveMeterPerRotation,
					Rotation2d.fromDegrees(swerveEncoders[i].getPosition()));
			swerveModuleState[i] = moduleState;
		}
		return swerveModuleState;
	}

	

	

	public void calibrateGyro() {
		gyroSensor.calibrate();
	}



	public void startHold() {
		//TODO
		driveState = DriveState.HOLD;
		configHigh();
	}

	public void endHold() {
		driveState = DriveState.TELEOP;
	}

	

	public void hold() {
		//TODO


		
	}



	public void swerveDrive(double x1, double x2, double y1){

		if(Math.abs(x1)<0.25) x1 = 0;
		if(Math.abs(x2)<0.25) x2 = 0;
		if(Math.abs(y1)<0.25) y1 = 0;
		swerveDrive(new ChassisSpeeds(Units.inchesToMeters(Constants.DriveHighSpeed)*x1,Units.inchesToMeters(Constants.DriveHighSpeed)*x2, y1*2));
		//System.out.println(x1 + ", "  + x2 + ", " + y1);

		

	}

	public void swerveDriveFeildRelitive(double x1, double x2, double y1){


		swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(Units.inchesToMeters(Constants.DriveHighSpeed)*x1,Units.inchesToMeters(Constants.DriveHighSpeed)*x2, y1*8, Rotation2d.fromDegrees(getAngle())));

	}

	int temp = 1;

	public double[] angleOffsets = new double[4];

		
	double doubleMod(double x, double y){
		// x mod y behaving the same way as Math.floorMod but with doubles
		return (x - Math.floor(x/y) * y);
	}	


	private void swerveDrive(ChassisSpeeds chassisSpeeds){

		/*Things to cahnge before using
		1. Ids
		2. set Locations of all wheels
		3. 
		*/

		//TODO: Set motor control modes

		


		synchronized (this) {
			driveState = DriveState.TELEOP;
		}

		SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
		//SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 0.1 ); //Units.inchesToMeters(Constants.DriveHighSpeed)

		System.out.println(moduleStates);
		boolean rotate = true;

		if(chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) rotate = false;

		for (int i = 0; i < 4; i++){
			SwerveModuleState tragetState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(swerveEncoders[i].getPosition()));
			//SwerveModuleState tragetState = moduleStates[i];
			double targetAngle = tragetState.angle.getDegrees();
			double currentAngle = swerveEncoders[i].getPosition();

			

			double anglediff = doubleMod((targetAngle - currentAngle)+180, 360)-180;

			
			
			if(Math.abs(anglediff)<1 || !rotate){
				swerveMotors[i].set(0);
			}else{
				swervePID[i].setReference(swerveEncoders[i].getPosition() + anglediff, ControlType.kPosition);
			}
			swerveDriveMotors[i].set((tragetState.speedMetersPerSecond/Constants.DriveHighSpeed)*50*(Math.min(1, Math.max(0, 1-(Math.abs(anglediff)/20)))));

			//System.out.println(i + ": " + tragetState.speedMetersPerSecond/Units.inchesToMeters(Constants.DriveHighSpeed)+ ", " + anglediff);
		}

		System.out.println(RobotTracker.getInstance().getPoseMeters());
		
	}
	
	private void configMotors() {
		//TODO
		
	}

	public void resetMotionProfile() {
		moveProfiler.reset();
	}

	public double getAngle() {
		return gyroSensor.getAngle();
	}

	// public double getDistance() {
	// 	//TODO
	// 	return null;
	// }

	public Rotation2D getGyroAngle() {
		// -180 through 180
		return Rotation2D.fromDegrees(gyroSensor.getAngle());
	}


	public double getSpeed() {

		//TODO
		return (Double) null;
	}



	public double scaleJoystickValues(double rawValue, int profile) {
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinControllerInput[profile],
				Constants.MaxControllerInput, Constants.MinControllerOutput, Constants.MaxControllerOutput),
				rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed, Rotation2D targetHeading) {
		driveState = DriveState.PUREPURSUIT;
		autonomousDriver = new PurePursuitController(autoPath, isReversed, targetHeading);
		autonomousDriver.resetTime();
		configAuto();
		//System.out.println("even more bad");
		updatePurePursuit();
	}

	public void setBrakeState(NeutralMode mode) {
		//leftTalon.setNeutralMode(mode);
		//rightTalon.setNeutralMode(mode);
		//leftSlaveTalon.setNeutralMode(mode);
		//rightSlaveTalon.setNeutralMode(mode);
		//leftSlave2Talon.setNeutralMode(mode);
		//rightSlave2Talon.setNeutralMode(mode);
	}

	public double getVoltage() {
		return 0;
		//return (leftTalon.getMotorOutputVoltage() + rightTalon.getMotorOutputVoltage()
		//		+ .getMotorOutputVoltage() + rightSlaveTalon.getMotorOutputVoltage()
		//		+ rightSlave2Talon.getMotorOutputVoltage() + leftSlave2Talon.getMotorOutputVoltage()) / 6;
	}

	public boolean hasStickyFaults()
	{
		short kCANRXmask = (short)(1 << CANSparkMax.FaultID.kCANRX.ordinal());
		short kCANTXmask = (short)(1 << CANSparkMax.FaultID.kCANTX.ordinal());
		short kHasResetmask = (short)(1 << CANSparkMax.FaultID.kHasReset.ordinal());
		short leftFaults = leftSpark.getStickyFaults();
		short rightFaults = rightSpark.getStickyFaults();
		//System.out.println("left: " + leftFaults + " right: " + rightFaults);


		boolean leftSticky = (leftFaults & kCANRXmask) != 0 || (leftFaults & kCANTXmask) != 0 || (leftFaults & kHasResetmask) != 0;
		boolean rightSticky = (rightFaults & kCANRXmask) != 0 || (rightFaults & kCANTXmask) != 0 || (rightFaults & kHasResetmask) != 0;
		return leftSticky || rightSticky;
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > Constants.DriveHighSpeed
				|| Math.abs(setVelocity.leftVelocity) > Constants.DriveHighSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.DriveHighSpeed + " !", false);
			return;
		}
		 //System.out.println("Left: " + setVelocity.leftVelocity);
		 //+ getLeftSpeed());
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.leftVelocity)/Constants.kDriveInchesPerSecPerRPM;
		double rightSetpoint = (setVelocity.rightVelocity)/Constants.kDriveInchesPerSecPerRPM;



		//leftTalon.set(ControlMode.Velocity, leftSetpoint);
		//rightTalon.set(ControlMode.Velocity, rightSetpoint);


		
		leftSparkPID.setReference(leftSetpoint, ControlType.kVelocity);
		rightSparkPID.setReference(rightSetpoint, ControlType.kVelocity);

		//System.out.println("desired left rpm: " + rightSetpoint + " desired right rpm: " + leftSetpoint);
		//System.out.println("actual left rpm: " + getLeftSpeed() + " actual right rpm: " + getRightSpeed());

	}

	public synchronized void setSimpleDrive(boolean setting) {
		if(drivePercentVbus != setting) System.out.println("Simple drive: " + setting);
		drivePercentVbus = setting;
	}

	public synchronized boolean getSimpleDrive() {
		return drivePercentVbus;
	}

	@Override
	public void update() {
	//	System.out.println("L speed " + getLeftSpeed() + " position x " + RobotTracker.getInstance().getOdometry().translationMat.getX());
	//	System.out.println("R speed " + getRightSpeed() + " position y " + RobotTracker.getInstance().getOdometry().translationMat.getY());
	//debugSpeed();
	//System.out.println(driveState);	
	DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
			case TELEOP:
			//System.out.println(leftSpark.getStickyFaults() + " slave faults " + leftSparkSlave.getStickyFaults());
		//CANSparkMax.FaultID.kCANTX
				
				break;
			case PUREPURSUIT:
				//System.out.println("bad!");
				updatePurePursuit();
				break;
			case TURN:
				updateTurn();
				break;
			case HOLD:
				hold();
				break;
		}
		
	}
	synchronized public boolean isAiming() {
		return isAiming; 
	}

	public void setRotation(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
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
		//TODO redo
		return false;
	}

	private void updateTurn() {
		double error = wantedHeading.inverse().rotateBy(RobotTracker.getInstance().getOdometry().rotationMat).getDegrees();
		double deltaSpeed;

		

		// System.out.println(Timer.getFPGATimestamp() - prevTime);
		// prevTime =  Timer.getFPGATimestamp(); 

		
		//System.out.println(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees());
		//System.out.println("error: " + error);
		if (rotateAuto){
			deltaSpeed = turnPIDAuto.update(error);
			deltaSpeed = Math.copySign(Math.max(Math.abs(deltaSpeed), 3), deltaSpeed);
		} else {
			deltaSpeed = turnPID.update(error);
			deltaSpeed = Math.copySign(Math.max(Math.abs(deltaSpeed), 4.5), deltaSpeed); //2.6
		}
		//System.out.println("error: "  + error + " DeltaSpeed: " + deltaSpeed);

		
		//System.out.println(deltaSpeed);
		//deltaSpeed = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, Constants.DriveHighSpeed), deltaSpeed);
		//System.out.println("error " + error + " speed " + (getLeftSpeed()-getRightSpeed()));

		//TODO Finish
	}


	private void updatePurePursuit() {
	//	System.out.println("updating pure presuit");
		ChassisSpeeds signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		if (autonomousDriver.isDone()) {
			synchronized (this) {
				driveState = DriveState.DONE;
			}
			configHigh();
		}
		//System.out.println("signal l:" + signal.command.leftVelocity + " signal R " + signal.command.rightVelocity);
		swerveDrive(signal);
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



	public double getLeftDistance() {
		//TODO
		return 0;
	}



	public double getDistance() {

		//TODO
		return 0;
	}
}
