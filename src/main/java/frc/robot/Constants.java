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
	public static final int DriveLeftMasterId = 3;
	public static final int DriveLeftSlave1Id = 4;
	public static final int DriveLeftSlave2Id = 14;//not currently used
	public static final int DriveRightMasterId = 5;
	public static final int DriveRightSlave1Id = 6;
	public static final int DriveRightSlave2Id = 13;//not currently used

	//Shooter constants
	public final static int ShooterMasterId = 0;
	public final static int ShooterSlaveId1 = 1;
	public final static int ShooterSlaveId2 = 2;
	public final static int ShooterSlaveId3 = 3;

	public static final int FeederMotorId = 15;
	public static final double kFeederP = 0.40;
	public static final double kFeederI = 0.0;
	public static final double kFeederD = 0.3;
	public static final int FeederIntegralZone = 1000;
	public static final int FeederMotorSpeed = 1000;

	public static final double kShooterP = 0.40;
	public static final double kShooterI = 0.0;
	public static final double kShooterD = 0.3;
	public static final double ShooterGain = 1;
	public static final int ShooterMaxDeviation = 100;

	public static final int HoodMotorId = 4;
	public static final double kHoodP = 0;
	public static final double kHoodI = 0;
	public static final double kHoodD = 0;
	public static final double kHoodF = 0;
	public static final int HoodRotationsConversion = (12/60)*(20/5760);//100;
	public static final int HoodMaxDeviation = 5;


	public static final int HatchIntakeMotorId = 23;
	public static final int HatchIntakeDeployMotorId = 22;

	public static final int ElevatorMasterId = 9;
	public static final int ElevatorSlaveId = 8;

	public static final int ManipulatorMotor1Id = 31;
	public static final int ManipulatorMotor2Id = 32;
	
	public static final int ClimberMasterId = 15;
	public static final int ClimberSlaveId = 16;

	//Intake Constants
	//public static final int IntakeDeployTime = 0.4; //Random number
	public static final int IntakeMasterId = 25;//Random number
	//public static final double IntakeDeployTime = 1.67;//Random number
	public static final int IntakeSolenoidId = 1;//Random number
	public static final double intakeForwardSpeed = 1;

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
	public static final double kDriveInchesPerSecPerRPM = 2 * Math.PI/60d * Constants.WheelDiameter/2d
	* 22d / 62d / 3d;
	public static final double maxTurnError = 2;
	public static final double maxPIDStopSpeed = 8;
	public static final double DriveHighSpeed = 190;
	public static final double DriveLowSpeed = 95;
	
	public static final double kDriveRightAutoP = 4*0.0005263 * kDriveInchesPerSecPerRPM; //0.00065
	public static final double kDriveRightAutoD = 0.000; 
	public static final double kDriveRightAutoF = 1/193.12283370478679  * kDriveInchesPerSecPerRPM; //0.055
	public static final double kDriveLeftAutoP = 4*0.0005263 * kDriveInchesPerSecPerRPM;
	public static final double kDriveLeftAutoD = 0.000; //0.0001
	public static final double kDriveLeftAutoF = 1/203.7763632654868 * kDriveInchesPerSecPerRPM ; //0.0005 too high

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
	public static final double IntakeMotorPowerIntake = 0.5; //0.5
	public static final double IntakeMotorPowerEject = 0.275;
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

	// Turret
	public static final int TurretCollisionRange = 0;
	public static final double maxTurretOverTravel = 20;
	public static final int turretLimitId = 8;//Random channel for now
	public static final int maxTurretHomingAngle = 45;//Random degrees for now
	public static final double turretHomingSpeed = 0.2;//Random percent for now
	public static final int TurretMotorId = 7;
//	public static final double kTurretP = 0.25;
	public static final double TurretTargetError = 3;
	public static final double AutoScoreDistance = 36 - 5.25;
	public static final double AutoScoreDistanceBallClose = 34.5 - 8;
	public static final double AutoScoreDistanceBallFar = 45 - 8;
	public static final double MaxVisionScoreAngle = 10;



	public static final int TurretMaxHomingAngle = 45;//Random degrees for now
	public static final double TurretHomingPower = 0.4;//Random percent for now
	public static final double kTurretP = 1.6;


	public static final double kTurretI = 0.00;
	public static final double kTurretD = 1.0;

	public static final double kTurretManual = 3.0;


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
	public static final double ClimberMaxAngle = 90; //207
	public static final double ClimberMinAngle = -40; //82;//82; //4
	public static final double ClimberStartAngle = 2; //no
	public static final int ClimberSolenoidID = 3;
	
	
	// Camera
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
	public static final int wheelSpinnerId = 99; //temp
	public static final double wheelSpinnerLevelTwoSpeed = 1.0;
	public static final double wheelSpinnerLevelThreeSpeed = 0.5; //temp
	public static final double colorConfirmCycles = 20;
	public static final int spinnerWheelReduction = 1/160;
	public static final int spinnerSolenoidID = 34; //temp
	public static final int ShooterIntegralZone = 0;

	//Update Period Constants
	public static final int intakePeriod = -1;

	

	


	
	
	private Constants() {
	}
}