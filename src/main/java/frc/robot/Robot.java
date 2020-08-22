// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.*;
import frc.subsystem.*;
import frc.subsystem.Hopper.FrontMotorState;
import frc.subsystem.Hopper.SnailMotorState;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.VisionManager.VisionStatus;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.concurrent.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import java.util.*;

import frc.utility.Controller;
import frc.utility.JetsonUDP;
import frc.utility.NavXMPX_Gyro;
import frc.utility.VisionTarget;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
@SuppressWarnings("unused")



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
  
  JetsonUDP jetsonUDP = JetsonUDP.getInstance();
  Drive drive = Drive.getInstance();
  RobotTracker robotTracker = RobotTracker.getInstance();
  Shooter shooter = Shooter.getInstance();
  Climber climber = Climber.getInstance();
  Hopper hopper = Hopper.getInstance();
  Intake intake = Intake.getInstance();
  VisionManager visionManager = VisionManager.getInstance();
  ControlPanel controlPanel = ControlPanel.getInstance();
  BlinkinLED blinkinLED = BlinkinLED.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(4);
  Thread auto;
  TemplateAuto option;
  AutoPosition autoPosition = AutoPosition.MIDDLE;

  boolean firstTeleopRun = true;
  boolean visionOff = false;

  boolean shooterSetOn = false;
  boolean intakeSetDeployed = true;
  double hoodPosition = 90;
  int shooterSpeed = 6000;
  // TODO:Remove if new control code works
  // boolean ejectAll = false;
  // boolean fireShooter = false;
  // boolean intakeOn = false;
  // boolean intakeEject = false;
  // boolean shooterOn = false;
  // boolean ejectShooter = false;
  // boolean hopperEject = false;
  // boolean hopperOn = false;
  int shooterMode = 0;

  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<String>();
  //private final SendableChooser<String> dir_chooser = new SendableChooser<String>();
  private final SendableChooser<String> goodBad = new SendableChooser<String>();
  private final SendableChooser<String> startChooser = new SendableChooser<String>();
  //private final SendableChooser<String> red_blue = new SendableChooser<String>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  enum AutoPosition {
    MIDDLE, LEFT, RIGHT
  }
  
  @Override
  public void robotInit() {
    light.set(Relay.Value.kOff);
    //light.set(false);
    //light.setRaw(0);
    
    Thread.currentThread().setPriority(5);
    drive.calibrateGyro();
    //m_chooser.addOption("Cargo F_F", "Cargo F_F");
    autoChooser.addOption("3 Ball", "3 Ball");
    autoChooser.addOption("3 Ball Drive", "3 Ball Drive");

    autoChooser.setDefaultOption("8 Ball", "8 Ball");
    
    SmartDashboard.putData("Autonomous Mode", autoChooser);

    

    startChooser.setDefaultOption("left", "left");
    startChooser.addOption("mid", "mid");
    startChooser.addOption("right", "right");

    SmartDashboard.putData("Starting Pos", startChooser);

  
    goodBad.setDefaultOption("good", "good");
    goodBad.addOption("bad","bad");
    goodBad.addOption("mindBuisness", "mindBuisness");
    goodBad.addOption("mInDbUiSnEsS", "mInDbUiSnEsS");
    SmartDashboard.putData("Good or Bad? To be or Not to Be?", goodBad);

    shooter.homeHood();

    //scheduler.schedule(drive, executor);
		//scheduler.schedule(elevator, executor);
    //scheduler.schedule(collisionManager, executor);
    //scheduler.schedule(jetsonUDP, executor);
    //scheduler.schedule(robotTracker, executor);
    

    // elevator.elevHome();
    drive.setSimpleDrive(false);

    Thread.currentThread().setPriority(7);
    jetsonUDP.start();
    blinkinLED.setColor(0.89);
  }

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
  //  VisionTarget[] t = jetsonUDP.getTargets();
   // if(t != null) System.out.println("Recieving: " + t[0].x + "," + t[0].y);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
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
    // controlPanel.start();
    hopper.start();
    intake.start();
    // blinkinLED.start()
    visionManager.start();
    jetsonUDP.start();
    robotTracker.start();
    drive.start();


    jetsonUDP.changeExp(true);
  }

  

  @Override
  public void autonomousInit() {
    startAll();
    shooter.setSpeed(0);

    robotTracker.resetOdometry();

    autoDone = false;

    int autoDir = 1;
    double startX = 94.6;



    

    //if(start_chooser.getSelected().equals("Lvl2")) startPos = 18+19-3;//-8;
    //autoPosition = AutoPosition.MIDDLE;
   // if(m_chooser.getSelected().equals("Cargo 1_2")&& red_blue.getSelected().equals("Red")) option = new Ship1_2Red(autoDir, startPos);
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
    option = new EightBallOppTrench(275);//TenBall(275);

    if(autoChooser.getSelected().equals("3 Ball")) option = new ShootOnly(startX);
    else if(autoChooser.getSelected().equals("3 Ball Drive")) option = new ShootAndMove(startX);

    

    auto = new Thread(option);
    auto.start();
    
  

    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //System.out.println("robot angle: " + robotTracker.getOdometry().rotationMat.getDegrees() + " drive state " + drive.driveState);
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
 //   	System.out.println("ashwin says: " + robotTracker.getOdometry().translationMat.getX() + ", " +  robotTracker.getOdometry().translationMat.getY() + "  heading: " +  robotTracker.getOdometry().rotationMat.getDegrees());

    
  
    //System.out.println(drive.get)
  } 

  public synchronized void killAuto() {
    if(option != null) {
      option.killSwitch();
    }

    //new Thread().
    if(auto != null) {
      //auto.interrupt();
      //while(!auto.isInterrupted());
      while(auto.getState() != Thread.State.TERMINATED);
   
      //elevator.setHeight(Math.max(elevator.getHeight(), Constants.HatchElevLow));
      drive.stopMovement();
      drive.setTeleop();
    }
    visionManager.stop();
   // visionManager.setState(Vision);
    
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
    //drive.stopMovement();
    firstTeleopRun = true;
    //drive.setTeleop();
    
    
    
  }
  

  int c = 0;
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

      ArrayList<Double> times = new ArrayList<Double>();
      
      if(profileTeleop) times.add(Timer.getFPGATimestamp());

      xbox.update();
      stick.update();
      buttonPanel.update();
      wheel.update();

      //shooter presets
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

      if(xbox.getRawAxis(2) > 0.5 || stick.getRawButton(1)){
        if(visionOff || stick.getRawButton(1)){
          shooter.setFiring(true);
          hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
          hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
          blinkinLED.setColor(0.77);
          
        } else if(xbox.getRawAxis(2) > 0.5){
          visionManager.setState(VisionStatus.WIN);

        } else shooter.setFiring(false);
        
      }
      
      else{
        visionManager.setState(VisionStatus.IDLE);
        drive.cheesyDrive(-xbox.getRawAxis(1),  xbox.getRawAxis(4),true);
        if (shooterMode == 1){
          blinkinLED.setColor(-0.29);
        } else if (shooterMode == 2){
          blinkinLED.setColor(-0.23);
        } else if (shooterMode == 3){
          blinkinLED.setColor(-0.15);
        }
      }
      
      

      //Turn Shooter On
      if (buttonPanel.getRawButton(5)){
        shooter.setSpeed(shooterSpeed);
        shooter.setHoodAngle(hoodPosition);

      } else if (!xbox.getRawButton(4)){
        shooter.setSpeed(0);

      }

      //Toggle Intake in and out
      if (xbox.getRisingEdge(2)){
        intakeSetDeployed = !intakeSetDeployed;
        intake.setDeployState(intakeSetDeployed ? DeployState.DEPLOY:DeployState.UNDEPLOY);

      }

      
      DeployState intakeDeployState = intake.getDeployState();
      if(visionManager.getState() == VisionManager.VisionStatus.IDLE && (true || DeployState.DEPLOY == intakeDeployState)) {
        if (buttonPanel.getRawButton(10)){
          //eject all
          intake.setIntakeState(IntakeState.EJECT);
          hopper.setFrontMotorState(FrontMotorState.REVERSE);
          hopper.setSnailMotorState(SnailMotorState.REVERSE , false);
          //ejectShooter = true; Was never used but was in code...

        } else if (buttonPanel.getRawButton(8)){
          intake.setIntakeState(IntakeState.EJECT);
          hopper.setFrontMotorState(FrontMotorState.ACTIVE);
          hopper.setSnailMotorState(SnailMotorState.ACTIVE , true);

        }else if (xbox.getRawButton(6)){
          intake.setIntakeState(IntakeState.EJECT);
          hopper.setFrontMotorState(FrontMotorState.INACTIVE);
          hopper.setSnailMotorState(SnailMotorState.INACTIVE, false);

        } else if(xbox.getRawAxis(3)>0.5){
          //intake on 
          intake.setIntakeState(IntakeState.INTAKE);
          hopper.setFrontMotorState(FrontMotorState.ACTIVE);
          hopper.setSnailMotorState(SnailMotorState.ACTIVE , true);

        }
      }

      if (stick.getRawButton(9) && stick.getRawButton(10)){
        climber.up();
      } else if (stick.getRawButton(12)) {
        climber.down();
      } else {
        climber.stop();
      }

      if(stick.getRawButton(7) && stick.getRawButton(8)) climber.release();
      
    }


  //Incase Things Break
  /*    fireShooter = false;
    ejectShooter = false;
    intakeOn = false;
    ejectAll = false;
    intakeEject = false;
    shooterOn = false;
    hopperEject = false;
    hopperOn = false;

    //System.out.println("ashwin says: " + robotTracker.getOdometry().translationMat.getX() + ", " +  robotTracker.getOdometry().translationMat.getY() + "  heading: " +  robotTracker.getOdometry().rotationMat.getDegrees());



     
      ArrayList<Double> times = new ArrayList<Double>();
      
      if(profileTeleop) times.add(Timer.getFPGATimestamp());


     // System.out.println("get hood angle " + shooter.getHoodAngle());
      // System.out.println(hopper.getCurrent());
      //System.out.println("turret " + turret.getAngle());
      //System.out.println(drive.getLeftSpeed() + " right: " + drive.getRightSpeed());
      xbox.update();
      stick.update();
      buttonPanel.update();
      wheel.update();

      if(xbox.getRawAxis(2) > 0.5 || stick.getRawButton(1)){
        if(visionOff || stick.getRawButton(1)){
          shooter.setFiring(true);
          hopper.setSnailMotorState(Hopper.SnailMotorState.ACTIVE, false);
          hopper.setFrontMotorState(Hopper.FrontMotorState.ACTIVE);
          hopperOn = true;
          System.out.println("nothin here");
          blinkinLED.setColor(0.77);
        } else if(xbox.getRawAxis(2) > 0.5){
          visionManager.setState(VisionStatus.WIN);
          //shooter.setFiring(false);
        } else shooter.setFiring(false);
        
      }
      
      else{
        visionManager.setState(VisionStatus.IDLE);
        drive.cheesyDrive(-xbox.getRawAxis(1),  xbox.getRawAxis(4),true);
        if (shooterMode == 1){
          blinkinLED.setColor(-0.29);
        } else if (shooterMode == 2){
          blinkinLED.setColor(-0.23);
        } else if (shooterMode == 3){
          blinkinLED.setColor(-0.15);
        }
      }
      

      if (xbox.getRisingEdge(2)){
        intakeSetDeployed = !intakeSetDeployed;

        intake.setDeployState(intakeSetDeployed ? DeployState.DEPLOY:DeployState.UNDEPLOY);
      }

      if(buttonPanel.getRawButton(5)) shooterSetOn = true; 
      else shooterSetOn = false; 

      //shooter presets
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
        //
        hoodPosition = 65; //TODO: Adjust numbers, 65
        shooterSpeed = 5000;//3250;,  5500
        visionOff = true;
        shooterMode = 3;

      }

      if (shooterSetOn){
        //Shoter button pushed
        shooterOn = true;
        
      }

      
      // else {
      //   fireShooter = false;
      //   hopperOn = false;
      // }

      if(xbox.getRawAxis(3)>0.5){
        //intake on 
        intakeOn = true;
        hopperOn = true;

      }

      if (xbox.getRawButton(6)){
        intakeEject = true;
        hopperOn = false;
      }

      if (buttonPanel.getRawButton(8)){
        //intake out
        intakeEject = true;
        hopperOn=true;
       // hopperEject = true;

        //hopperOn = false;
        intakeOn = false;

      }

      if (buttonPanel.getRawButton(10)){
        //eject all
        hopperOn = false;
        intakeOn = false;

        intakeEject = true;
        hopperEject = true;
        ejectShooter = true;


      }
      if (stick.getRawButton(9) && stick.getRawButton(10)){
        climber.up();
      } else if (stick.getRawButton(12)) {
        climber.down();
      } else {
        climber.stop();
      }

      if(stick.getRawButton(7) && stick.getRawButton(8)) climber.release();
      
      if(visionManager.getState() == VisionManager.VisionStatus.IDLE) {
      DeployState intakeDeployState = intake.getDeployState();
      if (true || DeployState.DEPLOY == intakeDeployState){
        if(hopperOn){
          if(intakeOn){
            hopper.setFrontMotorState(FrontMotorState.ACTIVE);
            hopper.setSnailMotorState(SnailMotorState.ACTIVE , true);
          } else {
            hopper.setFrontMotorState(FrontMotorState.ACTIVE);
            hopper.setSnailMotorState(SnailMotorState.ACTIVE , false);
          }

  
        } else if(hopperEject){
          hopper.setFrontMotorState(FrontMotorState.REVERSE);
          hopper.setSnailMotorState(SnailMotorState.REVERSE , false);
  
        } else {
          hopper.setFrontMotorState(FrontMotorState.INACTIVE);
          hopper.setSnailMotorState(SnailMotorState.INACTIVE, false);
  
        }
      
        if (intakeOn){
          intake.setIntakeState(IntakeState.INTAKE);

          
        } else if(intakeEject){
          intake.setIntakeState(IntakeState.EJECT);

        } else{
          intake.setIntakeState(IntakeState.OFF);

        }
  

      } else {
        intake.setIntakeState(IntakeState.OFF);
        hopper.setFrontMotorState(FrontMotorState.INACTIVE);
        hopper.setSnailMotorState(SnailMotorState.INACTIVE, false);

      }

      if (shooterOn){
        shooter.setSpeed(shooterSpeed);
        shooter.setHoodAngle(hoodPosition);
      } else if (!xbox.getRawButton(4)){
        shooter.setSpeed(0);
      }
    }

  }
  */

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
    


    
    shooter.pause();
    climber.pause();
    //controlPanel.pause();
    hopper.pause();
    intake.pause();
  }
  
  @Override
  public void disabledPeriodic() {
    //System.out.println(turret.turretHallEffect.get());
    //System.out.println(shooter.getHomeSwitch());
    try {
     // System.out.println(JetsonUDP.getInstance().getTargets()[0].x);
     // System.out.println(JetsonUDP.getInstance().getTargets()[0].distance);
    } catch(Exception e) {
      //System.out.println("cant get vision");
    }
  }

}
