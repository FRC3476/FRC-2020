// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;

public final class Constants {

	public static final boolean steeringWheel = false;
	// Networking
	public static final int TelemetryPort = 5810;
	public static final String DriverStationIPv4 = "10.34.76.5"; // Temporary
	public static final int JetsonPort = 5805;
	public static final String JetsonIPv4 = "10.34.76.8";
	public static final double VisionXOffset = 1.25;

	//Vision (cm)
	public static final double CamerTargetHeightOffset = 79.1608;
	public static final double CameraYAngle = 33.3679;



	// CAN IDs
	public static final int DriveLeftMasterId = 10;
	public static final int DriveLeftSlave1Id = 11;
	//public static final int DriveLeftSlave2Id = 14;//not currently used
	public static final int DriveRightMasterId = 12;
	public static final int DriveRightSlave1Id = 13;
	//public static final int DriveRightSlave2Id = 13;//not currently used

	//Swerve Constants
	public static final int DriveLeftFrontId = 10;
	public static final int DriveLeftBackId = 11;
	public static final int DriveRightFrontId = 12;
	public static final int DriveRightBackId = 13;
	public static final int DriveLeftFrontSwerveId = 14;
	public static final int DriveLeftBackSwerveId = 15;
	public static final int DriveRightFrontSwerveId = 16;
	public static final int DriveRightBackSwerveId = 17;

	// Locations for the swerve drive modules relative to the robot center.
	public static final Translation2d LeftFrontLocation = new Translation2d(0.381, 0.381);
	public static final Translation2d LeftBackLocation = new Translation2d(0.381, -0.381);
	public static final Translation2d RightFrontLocation = new Translation2d(-0.381, 0.381);
	public static final Translation2d RightBackLocation = new Translation2d(-0.381, -0.381);

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
	public static final double kShooterP = 2.0e-3;//1.0e-6;//1.0e-6; //1.5 e-4 //2.0e-3
	public static final double kShooterI = 8.0e-5;//5.0e-6;//5.0e-5;//5.0e-5;
	public static final double kShooterD = 2.0e-1;//10.5;//2.5e-5; 2.0e-1
	public static final double kShooterF = (0.5*1023.0) * ShooterRPMPerTicksPer100ms / 3270d;
	public static final double ShooterGain = 1;
	public static final int ShooterIntegralZone = (int)(200.0 / ShooterRPMPerTicksPer100ms);
	public static final int HomeSwitchId = 0; 

	public static final double ShooterMaxDeviation = 100;
	public static final double AutoShooterAccptableRange = 100;
	public static final double ShooterVisionMinimumTargetArea = 0.00; //TODO: Set

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
	public final static double AutoShooterOnTimePerBall = .40; //0.5, 0.35
	public final static boolean HoodMotorDirection = false; //True is Inverted 
	public final static double MaxHoodReleaseAngle = 75;
	public final static double MinHoodReleaseAngle = 12.32;

	//Intake Constants
	public static final int IntakeMasterId = 50;
	public static final int IntakeSolenoidId = 1;
	public static final double IntakeMotorPower = 1.0;
	public static final double IntakeOpenTime = 0.5;
	
	//Hopper Constants
	public static final int FrontHopperMotorId = 40; //random number
	public static final int SnailMotorId = 41; //random number
	public static final double HopperFrontMotorSpeed = 0.5;
	public static final double HopperSnailSpeed = 1.0;
	public static final double SlowHopperSnailSpeed = .5;
	
	
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
	
	// Autonomous Driving
	public static final double TrackRadius = -7; //-7;
	public static final double WheelDiameter = 6.0; //6.09; //expiermental
	public static final double MinTurningRadius = 40;
	public static final double MinPathSpeed = 20; 
	public static final double MaxPathSpeed = 140; //120
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 14;//30
	
	// Subsystems
	public static final int TimeoutMs = 10;
	
	// Drive
	public static final double kDriveInchesPerSecPerRPM = Math.PI* Constants.WheelDiameter/60d/10d;
	public static final double kDriveInchesPerRevolution = Math.PI* Constants.WheelDiameter/10d;

	public static final double maxTurnError = 1;
	public static final double maxPIDStopSpeed = 9;
  
	public static final double DriveHighSpeed = 145;
	
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
	
	public static final double kDriveLeftHighP = kDriveLeftAutoP;
	public static final double kDriveLeftHighD = kDriveLeftAutoD;
	public static final double kDriveLeftHighF = kDriveLeftAutoF;
	public static final double kDriveLeftHighFIntercept = 0;
	public static final double kDriveLeftHighA = 0;
	public static final double kHoldP = 4;


	public static final double DriveTeleopAccLimit = 500;
	public static final double DriveTeleopJerkLimit = 5000;
	public static final double DriveExpectedCurrent = 1.5;
	public static final double DriveExpectedRPM = 0;
	public static final double DriveExpectedPosition = 0;


	// Superstructure
	
	public static final int ClimberMotorID = 60;
	public static final double kClimberP = .10;
	public static final double kClimberI = .10;
	public static final double kClimberD = .10;
	public static final int ClimberIntergralZone = 10;
	public static final double climberIdleSpeed = 0;
	public static final double climberClimbSpeed = -1.00;

	//public static final double ClimberBottomHeight = 0;
	public static final double ClimberMaxTarget = -508000 * 0.9; //-339032.0*0.9 * (15/12)*(15/12);
	public static final double ClimberClimbedHeight = ClimberMaxTarget*2.2;
	
	
	// Camera
	public static final double CameraXFOV = 29.61;
	//PRAC
	//public static final double cameraYOffset = 5.310 + 1.25;//5.310 + 1.25;
	//public static final double cameraXOffset = -4.815 + 1.6 - 1.0 - 1; //-4.815 + 1.6 - 1.0

	//COMP
	public static final double cameraYOffset = 6.36;//5.310 + 1.25;//5.310 + 1.25;
	public static final double cameraXOffset = -1.22;//-4.815 + 1.6 - 1.0;



	//Color Wheel
	public static final char[] colorWheelOrder = {'R', 'Y', 'B', 'G', 'R', 'Y', 'B', 'G'};

	public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;
	public static final Color kBlueTarget = ColorMatch.makeColor(0.219, 0.465, 0.315); //0.113, 0.422, 0.463
	public static final Color kGreenTarget = ColorMatch.makeColor(0.236, 0.483, 0.280); //0.163, 0.580, 0.256
	public static final Color kRedTarget = ColorMatch.makeColor(0.269, 0.463, 0.265); // 0.521, 0.345, 0.133
	public static final Color kYellowTarget = ColorMatch.makeColor(0.276, 0.489, 0.233); //0.312, 0.564, 0.122
	public static final Color kWhiteTarget = ColorMatch.makeColor(0.250, 0.483, 0.265); //0.250, 0.483, 0.265

	public static final int wheelSpinnerId = 58; 
	public static final double wheelSpinnerLevelTwoSpeed = 1.0;
	public static final double wheelSpinnerLevelThreeSpeed = 0.5; 
	public static final double colorConfirmCycles = 10;

	public static final int spinnerWheelReduction = 1/160;
	public static final int spinnerSolenoidID = 3; 
	

	public static final int LevelThreeColorOffset = 2;

	//Update Period Constants
	public static final int intakePeriod = 50;
	public static final int controlPanelPeriod = 50;
	public static final int hopperPeriod = 50;
	public static final int DrivePeriod = 5;
	public static final int JetsonUdpPeriod = 5;
	public static final int RobotTrackerPeriod = 5;
	public static final int VisionManagerPeriod = 12;
	public static final int ShooterPeriod = 50;
	public static final int ClimberPeriod = 50; 


	public static final int BlinkinLEDPeriod = 50;
	
	private Constants() {
	}
}