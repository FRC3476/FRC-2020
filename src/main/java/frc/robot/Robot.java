// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.*;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.ScriptAutonomousStep;
import frc.subsystem.*;
import frc.subsystem.Hopper.FrontMotorState;
import frc.subsystem.Hopper.SnailMotorState;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.VisionManager.VisionStatus;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.*;

import java.text.DecimalFormat;
import java.util.*;
import java.util.ResourceBundle.Control;

import frc.utility.Controller;
import frc.utility.Limelight;
import frc.utility.ShooterPreset;
import frc.utility.VisionLookUpTable;
import frc.utility.Limelight.LedMode;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


	public static final boolean profileTeleop = true;

	
	//CollisionManager collisionManager = CollisionManager.getInstance();
	public Controller xbox = new Controller(0);
	public Controller wheel = new Controller(3);
	//public static Joystick xbox = new Joystick(0);
	public Controller stick = new Controller(1);
	public Controller buttonPanel = new Controller(2);
	Relay light = new Relay(3);
	//DigitalOutput light = new DigitalOutput(2);
	//PWM light = new PWM(0);

	Drive drive = Drive.getInstance();
	RobotTracker robotTracker = RobotTracker.getInstance();
	Shooter shooter = Shooter.getInstance();
	Climber climber = Climber.getInstance();
	Hopper hopper = Hopper.getInstance();
	Intake intake = Intake.getInstance();
	VisionManager visionManager = VisionManager.getInstance();
	ControlPanel controlPanel = ControlPanel.getInstance();
	BlinkinLED blinkinLED = BlinkinLED.getInstance();
	Limelight limelight = Limelight.getInstance();
	VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();

	ExecutorService executor = Executors.newFixedThreadPool(4);
	Thread auto;
	TemplateAuto option;
	AutoPosition autoPosition = AutoPosition.MIDDLE;

	boolean firstTeleopRun = true;
	boolean visionOff = false;
	boolean controlPanelDeployed = false;

	private boolean shooterSetOn = false;
	private boolean intakeSetDeployed = true;
	private double hoodPosition = 90;
	private int shooterSpeed = 6000;
	private int shooterMode = 0;
	private boolean targetFound = false;
	private boolean takeSnapshots = false;

	private String autoSelected;
	private final SendableChooser<String> autoChooser = new SendableChooser<String>();
	private final SendableChooser<String> networkAutoEnabled = new SendableChooser<String>();
	private final SendableChooser<String> red_blue = new SendableChooser<String>();
	private final SendableChooser<String> startChooser = new SendableChooser<String>();

	NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable autoDataTable = instance.getTable("autodata");
	NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");
	
	NetworkTable position = autoDataTable.getSubTable("position");
	NetworkTableEntry xPos = position.getEntry("x");
	NetworkTableEntry yPos = position.getEntry("y");
	NetworkTableEntry enabled = autoDataTable.getEntry("enabled");
	NetworkTableEntry processing = autoDataTable.getEntry("processing");


	enum AutoPosition {
		MIDDLE, LEFT, RIGHT
	}

	TrenchDashRed trenchDashRed = new TrenchDashRed();
	OpponentStealRed opponentTrenchRed = new OpponentStealRed();
	CenterBallsOnlyRed centerOnlyRed = new CenterBallsOnlyRed();

	TrenchDashBlue trenchDashBlue = new TrenchDashBlue();
	OpponentStealBlue opponentTrenchBlue = new OpponentStealBlue();
	CenterBallsOnlyBlue centerOnlyBlue = new CenterBallsOnlyBlue();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		light.set(Relay.Value.kOff);
		
		Thread.currentThread().setPriority(5);
		drive.calibrateGyro();
		autoChooser.addOption("3 Ball", "3 Ball");
		autoChooser.setDefaultOption("3 Ball Drive", "3 Ball Drive");
		autoChooser.addOption("Trench Dash", "Trench Dash");
		autoChooser.addOption("Opponent Trench", "Opponent Trench");
		autoChooser.addOption("Center Only", "Center Only");	
		SmartDashboard.putData("Autonomous Mode", autoChooser);


		networkAutoEnabled.setDefaultOption("Network Auto Enabled", "Network Auto Enabled");
		networkAutoEnabled.addOption("Network Auto Disabled", "Network Auto Disabled");
		SmartDashboard.putData("Network Auto Enabled/Disabled", networkAutoEnabled);

		red_blue.setDefaultOption("Red", "Red");
		red_blue.addOption("Blue", "Blue");
		SmartDashboard.putData("Side", red_blue);

		startChooser.setDefaultOption("left", "left");
		startChooser.addOption("mid", "mid");
		startChooser.addOption("right", "right");
		SmartDashboard.putData("Starting Pos", startChooser);

		shooter.homeHood();
		

		drive.setSimpleDrive(false);

		Thread.currentThread().setPriority(7);
		blinkinLED.setColor(0.89);
		limelight.setLedMode(LedMode.OFF);

		processing.setDouble(0);
		// autoPath.addListener((event) -> {

		// }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
	

		
	}

	NetworkAuto networkAuto;

	/**
	 * This function is called every robot packet, no matter the mode. Use
	 * this for items like diagnostics that you want ran during disabled,
	 * autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		//System.out.println("Distance: " + Limelight.getInstance().getDistance() + " tx: " + Limelight.getInstance().getHorizontalOffset());
		//System.out.println("Odometry: " + robotTracker.getOdometry().translationMat + " rotation: " +  robotTracker.getOdometry().rotationMat);
		if(isEnabled()){
			xPos.setDouble(robotTracker.getOdometry().translationMat.getX());
			yPos.setDouble(robotTracker.getOdometry().translationMat.getY());
		}

	
	}

	/**
	 * This autonomous (along with the chooser cod
	 * e above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	boolean autoDone;

	

	public void startAll()
	{
		light.set(Relay.Value.kOn);
	 // light.set(true);
		//light.setRaw(255);

		shooter.start();
		shooter.setSpeed(0);
		climber.start();
		controlPanel.start();
		hopper.start();
		intake.start();
		// blinkinLED.start()
		visionManager.start();
		robotTracker.start();
		drive.start();
	}



	@Override
	public void autonomousInit() {
		startAll();
		shooter.setSpeed(0);

		//robotTracker.resetOdometry();

		autoDone = false;
		limelight.setLedMode(LedMode.ON);

		int autoDir = 1;
		double startX = 94.6;
		enabled.setBoolean(true);

		switch(startChooser.getSelected()){
			case "mid":
				startX = 67;
				break;
			case "left":
				startX = 67-48;
				break;
			case "right":
				startX = 67+48;
				break;
				
		}

		//option = new ShootAndMove(startX);
		if(networkAuto == null || networkAutoEnabled.getSelected().equals("Network Auto Disabled")){
			if(red_blue.getSelected().equals("Red")){
				System.out.println("Using normal autos: Red Side");
	
				if(autoChooser.getSelected().equals("3 Ball")) option = new ShootOnly(startX);
				else if(autoChooser.getSelected().equals("3 Ball Drive")) option = new ShootAndMove(startX);
				else if(autoChooser.getSelected().equals("Trench Dash")) option = trenchDashRed;
				else if(autoChooser.getSelected().equals("Opponent Trench")) option = opponentTrenchRed;
				else if(autoChooser.getSelected().equals("Center Only")) option = centerOnlyRed;
			} else {
				System.out.println("Using normal autos: Blue Side");
	
				if(autoChooser.getSelected().equals("3 Ball")) option = new ShootOnly(startX);
				else if(autoChooser.getSelected().equals("3 Ball Drive")) option = new ShootAndMove(startX);
				else if(autoChooser.getSelected().equals("Trench Dash")) option = trenchDashBlue;
				else if(autoChooser.getSelected().equals("Opponent Trench")) option = opponentTrenchBlue;
				else if(autoChooser.getSelected().equals("Center Only")) option = centerOnlyBlue;
			}
			
			
			//option = trenchDash;
		} else {
			System.out.println("Using autos from network tables");
			option = networkAuto;
		}

		option.reset();

		auto = new Thread(option);
	
		auto.start();
		
	

		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		/*
		buttonPanel.update();
		if(!autoDone) {
		//for(int i = 1; i < 8; i++) {
			if(buttonPanel.getRawButton(2)) {
				autoDone = true;
				teleopInit();
			}
		//}
		}*/
		if(option.isFinished()) {
			teleopPeriodic();
		}	
	} 

	public synchronized void killAuto() {
		if(option != null) {
			option.killSwitch();
		}

		if(auto != null) {
			//auto.interrupt();
			//while(!auto.isInterrupted());
			while(auto.getState() != Thread.State.TERMINATED);
	 
			drive.stopMovement();
			drive.setTeleop();
		}
		visionManager.stop();
		
	}

	@Override 
	public void teleopInit() {
		startAll();
		blinkinLED.setColor(-0.23);
		killAuto();
		intake.setDeployState(DeployState.UNDEPLOY);
		if (intake.getDeployState() == DeployState.DEPLOY){
			intakeSetDeployed = true;
		} else{
			intakeSetDeployed = false;
		}

		hoodPosition = 65;
		shooterSpeed = 5000;
		
		System.out.println("teleop init!");
		drive.stopMovement();
		firstTeleopRun = true;
		//drive.setTeleop();
		robotTracker.resetOdometry();
		
		enabled.setBoolean(true);
		
	}
	

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
			//System.out.println("angle to target " + (drive.getGyroAngle().getDegrees()+limelight.getHorizontalOffset()));
			//ShooterPreset shooterPreset = visionLookUpTable.getShooterPreset(limelight.getDistance());
			//System.out.println("distance: " + limelight.getDistance() + " flywheel speed: " + shooterPreset.getFlyWheelSpeed() + " hood angle: " + shooterPreset.getHoodEjectAngle());

			

			ArrayList<Double> times = new ArrayList<Double>();
			
			if(profileTeleop) times.add(Timer.getFPGATimestamp());


			xbox.update();
			stick.update();
			buttonPanel.update();
			wheel.update();

			//shooter presets (now is used as a fallback)
			if (buttonPanel.getRisingEdge(1)){
				hoodPosition = 25; 
				shooterSpeed = 5500;
				visionOff = false;
				shooterMode = 1;

			} else if (buttonPanel.getRisingEdge(2)){
				hoodPosition = 33;
				visionOff = false;
				shooterSpeed = 5700;
				shooterMode = 2;

			} else if (buttonPanel.getRisingEdge(3)){
				//hits bootom of 3pt: 64.5, 5000
				hoodPosition = 65;
				shooterSpeed = 5000;//3250;,  5500
				visionOff = true;
				shooterMode = 3;

			}
			
			//do we want to shoot
			if(xbox.getRawAxis(2)>0.5 || stick.getRawButton(1)){
				//do we not want aiming
				limelight.setLedMode(LedMode.ON);
				if(visionOff || stick.getRawButton(1)){
					shooter.setFiring(true);
					hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
					hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
					//blinkinLED.setColor(0.77);
					visionManager.setState(VisionStatus.IDLE);
					System.out.println("no aim");
				} else{
					//We want to do auto aiming (This should shoot by itself if no target is visble)
					visionManager.setState(VisionStatus.WIN);
					System.out.println("winning");
				}

			} else {
				visionManager.setState(VisionStatus.IDLE);
				shooter.setFiring(false);
				if(!buttonPanel.getRawButton(6)){
					limelight.setLedMode(LedMode.OFF);
				}
				
			}

			//do normal drive fuction if Vision is idle
			if(visionManager.getState().equals(VisionStatus.IDLE) || !limelight.isTargetVisiable()){
				if(controlPanelDeployed){	
					if(-xbox.getRawAxis(1)>0){ //TODO: Remove negative if the slow is going backwards instead of forwards
						drive.cheesyDrive(-xbox.getRawAxis(1)/3,  xbox.getRawAxis(4)/2 ,true);
					} else{
						drive.cheesyDrive(-xbox.getRawAxis(1),  xbox.getRawAxis(4)/2 ,true);
					}
					

				} else drive.cheesyDrive(-xbox.getRawAxis(1),  xbox.getRawAxis(4),true);
				if(!(shooter.getTargetSpeed() > 10)){
					if(!limelight.isConnected()){
						blinkinLED.setColor(0.61);
					} else {
						blinkinLED.setColor(0.77);
					} 
				}
				
			}


			//Turn Shooter Flywheel On with distance detection
			if (buttonPanel.getRawButton(6)){
				limelight.setLedMode(LedMode.ON);
				//check if target is visible and that vision is enabled. Then turn shooter on with correct settings based on our distance
				if(limelight.isTargetVisiable() && limelight.getTagetArea()>= Constants.ShooterVisionMinimumTargetArea && !visionOff   && limelight.isConnected()){
					ShooterPreset sp = visionLookUpTable.getShooterPreset(limelight.getDistance());
					//System.out.println("flywheel speed: " +sp.getFlyWheelSpeed() + " hood angle: " + sp.getHoodEjectAngle());
					shooter.setSpeed(sp.getFlyWheelSpeed());
					shooter.setHoodAngle(sp.getHoodEjectAngle());
					targetFound = true;
			
				// use manuel selection if a target is not found
				} else if(!targetFound){
					//System.out.println("using manuel contorls");
					//the !targetFound means we should not go into manuel mode if we previously found our target
					shooter.setSpeed(shooterSpeed); 
					shooter.setHoodAngle(hoodPosition);
				}

			//Turn shooter flywheel on with manuel settings 
			} else if(buttonPanel.getRawButton(5)){
				//System.out.println("using manuel contorls");
				shooter.setSpeed(shooterSpeed); 
				shooter.setHoodAngle(hoodPosition);
				
			} else {
				shooter.setSpeed(0); //Turns off shooter flywheel
				targetFound = false;
			}
			

			//Toggle Intake in and out
			if (xbox.getRisingEdge(2) || buttonPanel.getRisingEdge(7)){
				intakeSetDeployed = !intakeSetDeployed;
				intake.setDeployState(intakeSetDeployed ? DeployState.DEPLOY:DeployState.UNDEPLOY);

			}

			if(!(xbox.getRawAxis(2)>0.5 || buttonPanel.getRawButton(8))) {
				//We're not shooting so use normal intake/hopper controlls
				if (buttonPanel.getRawButton(10)){
					//eject all
					intake.setIntakeState(IntakeState.EJECT);
					hopper.setFrontMotorState(FrontMotorState.REVERSE);
					hopper.setSnailMotorState(SnailMotorState.REVERSE , false);

				} else if (buttonPanel.getRawButton(8)){
					intake.setIntakeState(IntakeState.EJECT);
					hopper.setFrontMotorState(FrontMotorState.ACTIVE);
					hopper.setSnailMotorState(SnailMotorState.ACTIVE , true);

				}else if (xbox.getRawButton(4)){
					intake.setIntakeState(IntakeState.EJECT);
					hopper.setFrontMotorState(FrontMotorState.ACTIVE);
					hopper.setSnailMotorState(SnailMotorState.ACTIVE, false);

				} else if(xbox.getRawAxis(3)>0.5){
					//intake on 
					intake.setIntakeState(IntakeState.INTAKE);
					hopper.setFrontMotorState(FrontMotorState.ACTIVE);
					hopper.setSnailMotorState(SnailMotorState.ACTIVE , true);

				} else{
					intake.setIntakeState(IntakeState.OFF);
					hopper.setFrontMotorState(FrontMotorState.INACTIVE);
					hopper.setSnailMotorState(SnailMotorState.INACTIVE , true);
				}
			}

			//Climber controlls
			if (stick.getRawButton(9) && stick.getRawButton(10)){
				climber.up();
			} else if (stick.getRawButton(12)) {
				climber.down();
			} else {
				climber.stop();
			}

			if(stick.getRawButton(7) && stick.getRawButton(8)) climber.release();

			// if(buttonPanel.getRisingEdge(9)){
			// 	controlPanelDeployed = !controlPanelDeployed;
			// 	//System.out.println("Control panel deploy: " + controlPanelDeployed);
			// 	if(controlPanelDeployed){
			// 		controlPanel.deploy();
			// 	} else{
			// 		controlPanel.unDeploy();
			// 	}
			// }

			// if(buttonPanel.getRisingEdge(11)){
			// 	controlPanel.doLevelTwoSpin();
			// }

			// if(buttonPanel.getRisingEdge(12)){
			// 	controlPanel.doLevelThreeSpin();;
			// }

			// if(buttonPanel.getFallingEdge(11)){
			// 	controlPanel.stopSpin();;
			// }

			// if(buttonPanel.getFallingEdge(12)){
			// 	controlPanel.stopSpin();
			// }

			if(buttonPanel.getRisingEdge(9)){
				takeSnapshots = !takeSnapshots;
				limelight.takeSnapshots(takeSnapshots);
				System.out.println("taking snapshots " + takeSnapshots );
			}


			

		}

	@Override
	public void testInit() {
	 // drive.stopMovement();
	 // scheduler.resume();
	 //light.setRaw(255);
	 startAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		blinkinLED.setColor(.57);
		stick.update();
		xbox.update();
		buttonPanel.update(); 
		if(stick.getRawButton(11)){
			climber.reset();
		} else {
			climber.stop();
		}
	}

	@Override
	public void disabledInit() {
		blinkinLED.setColor(.77);
		//light.set(false);
		light.set(Relay.Value.kOff);
		//light.setRaw(0);

		killAuto();
		drive.configBrake();
		
		//TODO: REMOVE FOLLOWING IF YOU HAVE ISSUES
		drive.configBrake();

		ScheduledExecutorService worker = Executors.newSingleThreadScheduledExecutor();
		Runnable task = new Runnable() {
			public void run() {
				drive.configBrake();
			}
		  };
		worker.schedule(task, 2, TimeUnit.MILLISECONDS);
		// --------UNTIL HERE-----------
		
		shooter.pause();
		climber.pause();
		controlPanel.pause();
		hopper.pause();
		intake.pause();
		enabled.setBoolean(false);
	}
	

	String lastAutoPath = null;
	@Override
	public void disabledPeriodic() {
		if(autoPath.getString(null) != null && !autoPath.getString(null).equals(lastAutoPath) ){
			System.out.println("start parsing");
			processing.setDouble(1);
			lastAutoPath = autoPath.getString(null);

			networkAuto = new NetworkAuto();
			System.out.println("done parsing");
			processing.setDouble(2);
			
		}

	}

}
