// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
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

	private boolean drivePercentVbus;

	private NavXMPX_Gyro gyroSensor;// = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	//private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private final SynchronousPid turnPID;
	private final SynchronousPid turnPIDAuto;
	private DriveState driveState;
	private RateLimiter moveProfiler;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;
	boolean rotateAuto = false; 


	double prevPositionL = 0;
	double prevPositionR = 0;

	private boolean isAiming = false; 

	double prevTime;

	private final LazyCANSparkMax leftFrontSpark, leftBackSpark, rightFrontSpark, rightBackSpark;

	private final LazyCANSparkMax leftFrontSparkSwerve, leftBackSparkSwerve, rightFrontSparkSwerve, rightBackSparkSwerve;
	private final SwerveDriveKinematics swerveKinematics;

	private final LazyCANSparkMax[] swerveMotors = new LazyCANSparkMax[4];
	private final LazyCANSparkMax[] swerveDriveMotors = new LazyCANSparkMax[4]; 
	private final CANEncoder[] swerveEncoders = new CANEncoder[4];

	private final PWM leftFrontSparkPwmEncoder, leftBackSparkPwmEncoder, rightFrontSparkPwmEncoder, rightBackSparkPwmEncoder;
	private final PWM[] swerveEncodersPWM = new PWM[4];

	CANPIDController[] swervePID = new CANPIDController[4];


	private Drive() {
		super(Constants.DrivePeriod);
		gyroSensor = new NavXMPX_Gyro(SPI.Port.kMXP);

		//Swerve Drive Motors
		leftFrontSpark = new LazyCANSparkMax(Constants.DriveLeftFrontId, MotorType.kBrushless);
		leftBackSpark = new LazyCANSparkMax(Constants.DriveLeftBackId, MotorType.kBrushless);
		rightFrontSpark = new LazyCANSparkMax(Constants.DriveRightFrontId, MotorType.kBrushless);
		rightBackSpark = new LazyCANSparkMax(Constants.DriveRightBackId, MotorType.kBrushless);

		leftFrontSparkSwerve = new LazyCANSparkMax(Constants.DriveLeftFrontSwerveId, MotorType.kBrushless);
		leftBackSparkSwerve = new LazyCANSparkMax(Constants.DriveLeftBackSwerveId, MotorType.kBrushless);
		rightFrontSparkSwerve = new LazyCANSparkMax(Constants.DriveRightFrontSwerveId, MotorType.kBrushless);
		rightBackSparkSwerve = new LazyCANSparkMax(Constants.DriveRightBackSwerveId, MotorType.kBrushless);

		leftFrontSparkPwmEncoder = new PWM(0);
		leftBackSparkPwmEncoder = new PWM(1);
		rightFrontSparkPwmEncoder = new PWM(2);
		rightBackSparkPwmEncoder = new PWM(3);

		swerveMotors[0] = leftFrontSparkSwerve;
		swerveMotors[1] = leftBackSparkSwerve;
		swerveMotors[2] = rightFrontSparkSwerve;
		swerveMotors[3] = rightBackSparkSwerve;

		swerveDriveMotors[0] = leftFrontSpark;
		swerveDriveMotors[1] = leftBackSpark;
		swerveDriveMotors[2] = rightFrontSpark;
		swerveDriveMotors[3] = rightBackSpark;

		swerveEncodersPWM[0] = leftFrontSparkPwmEncoder;
		swerveEncodersPWM[1] = leftBackSparkPwmEncoder;
		swerveEncodersPWM[2] = rightFrontSparkPwmEncoder;
		swerveEncodersPWM[3] = rightBackSparkPwmEncoder;
		
		for(int i = 0; i <4; i++){
			swerveEncoders[i] = swerveMotors[i].getEncoder();
			swerveEncoders[i].setPositionConversionFactor(8.1503);//8.1466);
			swerveMotors[i].getAnalog(AnalogMode.kAbsolute).setPositionConversionFactor(360/3.3);//105.88);
			
			swervePID[i] = swerveMotors[i].getPIDController();
			swervePID[i].setP(Constants.SwerveDrivekP);
			swervePID[i].setD(Constants.SwerveDrivekd);
			swervePID[i].setI(Constants.SwerveDrivekI);
			swervePID[i].setFF(Constants.SwerveDrivekf);

			swerveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);

			swerveEncodersPWM[i].setBounds(2000, 0, 1500, 0, 1000); 
			
		}

		calculateOffsets();

		swerveKinematics = new SwerveDriveKinematics(Constants.LeftFrontLocation, Constants.LeftBackLocation, 
			Constants.RightFrontLocation,Constants.RightBackLocation);
	
		configMotors();

		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		turnPID = new SynchronousPid(3.5, 0, 0.0, 0); //P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DriveHighSpeedIn/5, -Constants.DriveHighSpeedIn/5);
		turnPID.setSetpoint(0);
		turnPIDAuto = new SynchronousPid(1, 0, 0, 0); //P=1.0 OR 0.8
		turnPIDAuto.setOutputRange(Constants.DriveHighSpeedIn/8, -Constants.DriveHighSpeedIn/8);
		turnPIDAuto.setSetpoint(0);
		

		moveProfiler = new RateLimiter(Constants.DriveTeleopAccLimit);

		configHigh();
		configAuto();
	}


	public void calculateOffsets(){
		for(int i = 0; i <4; i++){
			// double offset = -swerveMotors[i].getAnalog(AnalogMode.kAbsolute).getPosition();
			double offset = swerveEncodersPWM[i].getPosition() * 360; // Asume we're getting a value between 0 and 1
			System.out.println(i + ": " + offset);
			System.out.println(swerveEncoders[i].setPosition(offset));
		
		}
	}

	public SwerveDriveKinematics getSwerveDriveKinematics(){
		return swerveKinematics;
	}

	public void setDriveState(DriveState driveState){
		this.driveState = driveState;
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

		driveMultiplier = Constants.DriveHighSpeedIn;
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

	public void swerveDrive(ControllerDriveInputs inputs){
		inputs.applyDeadZone(0.05, 0.05, 0.2, Constants.DriveStrafeDeadZone).squareInputs();
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(Constants.DriveHighSpeedM*inputs.getX(),Constants.DriveHighSpeedM*inputs.getY(), inputs.getRotation()*2);
		swerveDrive(chassisSpeeds);
	}

	public void swerveDriveFieldRelitive(ControllerDriveInputs inputs){
		inputs.applyDeadZone(0.05, 0.05, 0.2, Constants.DriveStrafeDeadZone).squareInputs();
		swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(Constants.DriveHighSpeedM*inputs.getX(), Constants.DriveHighSpeedM*inputs.getY(), inputs.getRotation()*2, getGyroAngle().getWPIRotation2d()));
	}

	int temp = 1;

		
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

		//System.out.println(moduleStates);
		boolean rotate = true;

		if(chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) rotate = false;

		for (int i = 0; i < 4; i++){
			SwerveModuleState tragetState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(swerveEncoders[i].getPosition()));
			//SwerveModuleState tragetState = moduleStates[i];
			double targetAngle = tragetState.angle.getDegrees();
			double currentAngle = swerveEncoders[i].getPosition();

			

			double anglediff = doubleMod((targetAngle - currentAngle)+180, 360)-180;

			//System.out.println(i + ": angle offset: " + "N/a" + ", curret angle: " + swerveEncoders[i].getPosition() + ", current anjusted angle: " + currentAngle + " set point: " + (swerveEncoders[i].getPosition() + anglediff));
			
			if(Math.abs(anglediff)<1 || !rotate){
				swerveMotors[i].set(0);
			}else{
				swervePID[i].setReference(swerveEncoders[i].getPosition() + anglediff, ControlType.kPosition);
			}
			//swerveDriveMotors[i].set((tragetState.speedMetersPerSecond/Constants.DriveHighSpeed)*50*(Math.min(1, Math.max(0, 1-(Math.abs(anglediff)/20)))));
			swerveDriveMotors[i].set(tragetState.speedMetersPerSecond/Constants.DriveHighSpeedM);
			//System.out.println(i + ": " + tragetState.speedMetersPerSecond/Units.inchesToMeters(Constants.DriveHighSpeed)+ ", " + anglediff);
			
		}
		
		//System.out.println(RobotTracker.getInstance().getPoseMeters());
		
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

	double autoStartTime;
	HolonomicDriveController controller = new HolonomicDriveController(
  		new PIDController(1, 0, 0), new PIDController(1, 0, 0),
  		new ProfiledPIDController(1, 0, 0,
    	new TrapezoidProfile.Constraints(6.28, 3.14)));
	Trajectory currentAutoTrajectory;

	public synchronized void setAutoPath(Trajectory trajectory) {
		driveState = DriveState.RAMSETE;
		this.currentAutoTrajectory = trajectory;
		autoStartTime = Timer.getFPGATimestamp();
		configAuto();
		configCoast();
		updateRamsete();
	}

	private void updateRamsete() {
		Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp()-autoStartTime);
		System.out.println(goal);
		ChassisSpeeds adjustedSpeeds = controller.calculate(RobotTracker.getInstance().getPoseMeters(), goal, Rotation2d.fromDegrees(0));
		swerveDrive(adjustedSpeeds);
		//System.out.println(ramseteController.atReference());
		//System.out.println("target speed" + Units.metersToInches(wheelspeeds.leftMetersPerSecond) + " " + Units.metersToInches(wheelspeeds.rightMetersPerSecond) + "time: " +(Timer.getFPGATimestamp()-autoStartTime) );
		//TODO: not working
		if(controller.atReference() && (Timer.getFPGATimestamp()-autoStartTime)>= currentAutoTrajectory.getTotalTimeSeconds()){
			driveState = DriveState.DONE;
			stopMovement();
		}
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
			case DONE:
				break;
			case RAMSETE:
				updateRamsete();
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
