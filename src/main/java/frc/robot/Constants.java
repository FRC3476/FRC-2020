// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;

public final class Constants {

	public static final boolean steeringWheel = false;
	// Networking
	public static final int TelemetryPort = 5810;
	public static final String DriverStationIPv4 = "10.34.76.5"; // Temporary
	public static final int JetsonPort = 5805;
	public static final String JetsonIPv4 = "10.34.76.8";


	// CAN IDs
	public static final int DriveLeftMasterId = 10;
	public static final int DriveLeftSlave1Id = 11;
	//public static final int DriveLeftSlave2Id = 14;//not currently used
	public static final int DriveRightMasterId = 12;
	public static final int DriveRightSlave1Id = 13;
	//public static final int DriveRightSlave2Id = 13;//not currently used

	//Shooter constants
	public final static int ShooterMasterId = 30;
	public final static int ShooterSlaveId1 = 31;
	public final static int ShooterSlaveId2 = 32;
	public final static int ShooterSlaveId3 = 33;
	

	public static final int FeederMotorId = 34;
	public static final double kFeederP = 0.40;
	public static final double kFeederI = 0.0;
	public static final double kFeederD = 0.3;
	public static final int FeederIntegralZone = 1000;
	public static final double FeederMotorSpeed = .5;

	public static final double ShooterRPMPerTicksPer100ms = 600d/2048d;
	public static final double kShooterP = 1.0e-5;
	public static final double kShooterI = 0;//5.0e-5;
	public static final double kShooterD = 1.0e-3;
	public static final double kShooterF = (0.5*1023) * ShooterRPMPerTicksPer100ms / 3238d;
	public static final double ShooterGain = 1;
	public static final int ShooterIntegralZone = (int)(200.0 / ShooterRPMPerTicksPer100ms);
	public static final int HomeSwitchId = 0; 

	public static final double ShooterMaxDeviation = 10000;

	public static final double TakeBackHalfGain = 2.0e-6;

	public static final int HoodMotorId = 35;
	public static final double kHoodP = 16.0e-1;
	public static final double kHoodI = 0;
	public static final double kHoodD = 0;
	public static final double kHoodF = 0;
	public static final double HoodRotationsPerDegree = (60d/12d)*(576d/20d)*(1d/360d);//(12/60)*(20/576);//100;
	public static final double HoodMaxDeviation = .2;
	public static final double HoodHomingSpeed = -0.4;
	public static final double HoodPIDSpeedMax = 0.8;
	public static final double hoodMotorStalledAmps = 5; //not used
	public static final double HoodHomeTimeout = 1.5;
	public final static double AutoShooterOnTime = 3;
	public final static boolean HoodMotorDirection = false; //True is Inverted 
	public final static double MaxHoodReleaseAngle = 75;
	public final static double MinHoodReleaseAngle = 12.32;

	public static final int HatchIntakeMotorId = 100;
	public static final int HatchIntakeDeployMotorId = 101;

	public static final int ElevatorMasterId = 200;
	public static final int ElevatorSlaveId = 201;

	public static final int ManipulatorMotor1Id = 210;
	public static final int ManipulatorMotor2Id = 211;
	
	public static final int ClimberMasterId = 220;
	public static final int ClimberSlaveId = 221;
	public static final int ClimberPeriod = -1; 

	//Intake Constants
	public static final int IntakeMasterId = 50;
	public static final int IntakeSolenoidId = 5;
	public static final double IntakeMotorPower = 1.0;
	public static final double IntakeOpenTime = 0.5;
	
	//Hopper Constants
	public static final int FrontHopperMotorId = 40; //random number
	public static final int SnailMotorId = 41; //random number
	public static final double HopperFrontMotorSpeed = 0.3;
	public static final double HopperSnailSpeed = 1.0;
	
	// PCM IDs
	public static final int DriveShifterSolenoidId = 4;
	public static final int BallIntakeSolenoidId = 7;
	public static final int ArmSolenoidId = 5;
	public static final int ManipulatorSolenoidId = 6;

	// IO IDs
	public static final int TurretLimitId = 0;
	
	// Controller

	public static final double[] MinControllerInput = {0.15, 0.08};
	public static final double MaxControllerInput = 1;
	public static final double MinControllerOutput = 0;
	public static final double MaxControllerOutput = 1;
	public static final double MaxAcceleration = 1000;
	
	// General
	public static final double EncoderTicksPerRotation = 4096;
	public static final double DegreesPerEncoderTick = 360 * (1d / EncoderTicksPerRotation);
	public static final double EncoderTicksPerDegree = (1d / 360) * EncoderTicksPerRotation;

	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
	public static final double ExpectedPositionTolerance = 0;

	// Game
	public static final double RocketBaseHeight = 27.5;
	public static final double RocketMiddleHeight = 55.5;
	public static final double RocketTopHeight = 83.5;

	public static final double HatchPanelHeight = 2 + (1 / 6); // The height of each hatch panel
	
	// Autonomous Driving
	public static final double TrackRadius = -12;
	public static final double WheelDiameter = 6.0; //6.09; //expiermental
	public static final double MinTurningRadius = 40;
	public static final double MinPathSpeed = 20; 
	public static final double MaxPathSpeed = 120; //120
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;
	
	// Subsystems
	public static final int TimeoutMs = 10;
	
	// Drive
	public static final double kDriveInchesPerSecPerRPM = Math.PI* Constants.WheelDiameter/60d/10d;
	public static final double kDriveInchesPerRevolution = Math.PI* Constants.WheelDiameter/10d;
	public static final double maxTurnError = 0.5;
	public static final double maxPIDStopSpeed = 1.0;
	public static final double DriveHighSpeed = 145;
	public static final double DriveLowSpeed = 95;
	
	public static final double kDriveRightAutoP = 4*0.0005263 * kDriveInchesPerSecPerRPM; //0.00065
	public static final double kDriveRightAutoD = 0.000; 
	public static final double kDriveRightAutoF = 1/149.89885385408667  * kDriveInchesPerSecPerRPM; //0.055
	public static final double kDriveLeftAutoP = 4*0.0005263 * kDriveInchesPerSecPerRPM;
	public static final double kDriveLeftAutoD = 0.000; //0.0001
	public static final double kDriveLeftAutoF = 1/148.46271036085238 * kDriveInchesPerSecPerRPM ; //0.0005 too high

	public static final double kDriveRightHighP = kDriveRightAutoP;
	public static final double kDriveRightHighD = kDriveRightAutoD;
	public static final double kDriveRightHighF = kDriveRightAutoF;
	public static final double kDriveRightHighFIntercept = 0;
	public static final double kDriveRightHighA = 0;
	public static final double kDriveRightLowP = 0;
	public static final double kDriveRightLowD = 0;
	public static final double kDriveRightLowF = 0;
	public static final double kDriveRightLowFIntercept = 0;
	public static final double kDriveRightLowA = 0;
	
	public static final double kDriveLeftHighP = kDriveLeftAutoP;
	public static final double kDriveLeftHighD = kDriveLeftAutoD;
	public static final double kDriveLeftHighF = kDriveLeftAutoF;
	public static final double kDriveLeftHighFIntercept = 0;
	public static final double kDriveLeftHighA = 0;
	public static final double kDriveLeftLowP = 0;
	public static final double kDriveLeftLowD = 0;
	public static final double kDriveLeftLowF = 0;
	public static final double kDriveLeftLowFIntercept = 0;
	public static final double kDriveLeftLowA = 0;
	public static final double kHoldP = 4;

	
	
	

	public static final double DriveTeleopAccLimit = 120;
	public static final double DriveTeleopJerkLimit = 2000;
	public static final double DriveExpectedCurrent = 1.5;
	public static final double DriveExpectedRPM = 0;
	public static final double DriveExpectedPosition = 0;


	// Superstructure

	// Ground Ball Intake	
	public static final double IntakeMediumRPM = 700; // Random number for now
	public static final double IntakeFastRPM = 700; // Random number for now
	public static final long IntakeDeployTime = 0;
	//public static final int RollerMotorId = 25;
	public static final int BallIntakeMasterId = 25;
	public static final double BallIntakeDeployTime = 0.4;

	// Hatch Intake
	public static final double HatchIntakeMotorPower = 1.0;//Just a random percent for now
	public static final double HatchHandoffAngle = 48;
	public static final double HatchStowAngle = 0;
	public static final double HatchIntakeAngle = 178;
	public static final double HatchTargetError = 10;

	public static final double kHatchP = 0.8;
	public static final double kHatchI = 0.00;
	public static final double kHatchD = 0.0;

	// Elevator
	public static final double ElevatorHomeSpeed = -0.1;
	public static final double ElevatorInchesPerMotorRotation = 8;
	//public static final double ElevatorTicksPerInch = 4096.0/(1.5*3.141592);//orange string, no screw
	//public static final double ElevatorTicksPerInch = 52481/(64.25-4.375);//black string, screw spool
	public static final double ElevatorTicksPerInch = (51056-1105)/(67.125-5.5);//2.3mm string, screw spool
	//public static final double ElevatorTicksPerInch = 57378/(64.25-4.375); //practice bot
	public static final int ElevatorSensorPidIdx = 0;
	public static final double ElevatorTargetError = 2;
	
	public static final double ElevatorLowAmps = 0;
	public static final double ElevatorHighAmps = 25;
	public static final double ElevatorStallAmps = 0.25;
	
	public static final int ELevatorIntegralZone = 1000;
	public static final double kElevatorP = 0.40;
	public static final double kElevatorI = 0.0;
	public static final double kElevatorD = 0.3;
	
	public static final double ElevatorIntakeSafe = 10.5;
	public static final double ElevatorDeployingSafe = 0;
	public static final double ElevatorSafetyError = 0;
	public static final double ElevatorPositionDefault = 1 + (7 / 12);
	public static final double ElevatorPositionMiddle = ElevatorPositionDefault + HatchPanelHeight;
	public static final double ElevatorPositionHigh = ElevatorPositionDefault + (2 * HatchPanelHeight);
	public static final double ElevatorPositionLow = 0;
	
	public static final double ElevatorMaxHeight = 70;//in number for now
	public static final double ElevatorIntakeHeight = 10;//For now

	public static final double kElevatorManual = 1;


	//setpoints comp
	
	public static final double BallElevHigh = 60.8-1.0; //60.8
	public static final double BallElevMid = 33.6-0.5;
	public static final double BallElevLow = 5.8-0.5;
	public static final double BallElevCargo = 27;
	public static final double BallElevCargoGroundIntake = 0;
	public static final double BallHP = 20.85;

	/*
	public static final double BallElevHigh = 62.13; //60.8
	public static final double BallElevMid = 36.1;
	public static final double BallElevLow = 5.8-0.5;
	public static final double BallElevCargo = 27;
	public static final double BallElevCargoGroundIntake = 0;	
	public static final double BallHP = 20.85;
	*/
	public static final double HatchElevHigh = 56.8; //56.8
	public static final double HatchElevMid = 29.7;
	public static final double HatchElevLow = 1.5;//2.2;
	public static final double HatchHP = 1.2;

	public static final double ElevClearance = 7;
	

	// Manipulator
	public static final double ManipulatorNormalPower = 0.40;
	public static final double ManipulatorLowPower = 0.50;
	public static final double HandoffHoldTime = 0;
	
	// Climber
	public static final int ClimberSolenoidID = 3;
	public static final int ClimberMotorID = 3;
	public static final double kClimberP = .10;
	public static final double kClimberI = .10;
	public static final double kClimberD = .10;
	public static final int ClimberIntergralZone = 10;

	public static final int ClimberMaxTarget = 10000;
	public static final int ClimberClimbedHeight = 5000;
	public static final int ClimberHookingMaxVel = 100;
	public static final int ClimberHookingMaxAcel = 100;
	
	
	// Camera
	public static final double CameraXFOV = 29.61;
	//PRAC
	//public static final double cameraYOffset = 5.310 + 1.25;//5.310 + 1.25;
	//public static final double cameraXOffset = -4.815 + 1.6 - 1.0 - 1; //-4.815 + 1.6 - 1.0

	//COMP
	public static final double cameraYOffset = 6.36;//5.310 + 1.25;//5.310 + 1.25;
	public static final double cameraXOffset = -4.75;//-4.815 + 1.6 - 1.0;



	//Color Wheel
	public static final char[] colorWheelOrder = {'R', 'Y', 'B', 'G', 'R', 'Y', 'B', 'G'};

	public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;
	public static final Color kBlueTarget = ColorMatch.makeColor(0.219, 0.465, 0.315); //0.113, 0.422, 0.463
	public static final Color kGreenTarget = ColorMatch.makeColor(0.236, 0.483, 0.280); //0.163, 0.580, 0.256
	public static final Color kRedTarget = ColorMatch.makeColor(0.269, 0.463, 0.265); // 0.521, 0.345, 0.133
	public static final Color kYellowTarget = ColorMatch.makeColor(0.276, 0.489, 0.233); //0.312, 0.564, 0.122
	public static final Color kWhiteTarget = ColorMatch.makeColor(0.250, 0.483, 0.265); //0.250, 0.483, 0.265

	public static final int wheelSpinnerId = 99; 
	public static final double wheelSpinnerLevelTwoSpeed = 1.0;
	public static final double wheelSpinnerLevelThreeSpeed = 0.5; 
	public static final double colorConfirmCycles = 20;

	public static final int spinnerWheelReduction = 1/160;
	public static final int spinnerSolenoidID = 7; 
	

	public static final int LevelThreeColorOffset = 2;

	//Update Period Constants
	public static final int intakePeriod = 50;
	public static final int controlPanelPeriod = 50;
	public static final int hopperPeriod = 50;
	public static final int DrivePeriod = 5;
	public static final int JetsonUdpPeriod = 5;
	public static final int RobotTrackerPeriod = 5;
	public static final int VisionManagerPeriod = 12;
	public static final int ShooterPeriod = 10;


	public static final int BlinkinLEDPeriod = 50;
//=======

// c349e488ce1535e2388178b24ceb2496f2a1bdd1

	

	


	
	
	private Constants() {
	}
}