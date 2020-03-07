// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.concurrent.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import java.util.*;

import frc.utility.ThreadScheduler;
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
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;
  TemplateAuto option;
  AutoPosition autoPosition = AutoPosition.MIDDLE;

  boolean firstTeleopRun = true;

  boolean shooterSetOn = false;
  boolean intakeSetDeployed = false;
  double hoodPosition = 90;
  int shooterSpeed = 6000;
  boolean ejectAll = false;
  boolean fireShooter = false;
  boolean intakeOn = false;
  boolean intakeEject = false;
  boolean shooterOn = false;
  boolean ejectShooter = false;
  boolean hopperEject = false;
  boolean hopperOn = false;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<String>();
  private final SendableChooser<String> dir_chooser = new SendableChooser<String>();
  private final SendableChooser<String> goodbad = new SendableChooser<String>();
  private final SendableChooser<String> start_chooser = new SendableChooser<String>();
  private final SendableChooser<String> red_blue = new SendableChooser<String>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  enum AutoPosition {
    MIDDLE, LEFT, RIGHT
  }
  
  @Override
  public void robotInit() {
    Thread.currentThread().setPriority(5);
    drive.calibrateGyro();
    //m_chooser.addOption("Cargo F_F", "Cargo F_F");
    m_chooser.addOption("Cargo F_1", "Cargo F_1");
    m_chooser.addOption("Cargo 1_2", "Cargo 1_2");
    m_chooser.addOption("Rocket Mid Adaptive", "Rocket Mid Adaptive");

    m_chooser.setDefaultOption("Rocket Mid", "Rocket Mid");
    
    SmartDashboard.putData("Autonomous Mode", m_chooser);

    dir_chooser.setDefaultOption("Left", "Left");
    dir_chooser.addOption("Right", "Right");
    SmartDashboard.putData("Starting Side", dir_chooser);

    start_chooser.setDefaultOption("Lvl1", "Lvl1");
    start_chooser.addOption("Lvl2", "Lvl2");
    SmartDashboard.putData("Starting Height", start_chooser);

    red_blue.setDefaultOption("Red", "Red");
    red_blue.addOption("Blue", "Blue");
    SmartDashboard.putData("Red and Blue", red_blue);

    goodbad.setDefaultOption("good", "good");
    goodbad.addOption("bad","bad");
    goodbad.addOption("mindBuisness", "mindBuisness");
    goodbad.addOption("mInDbUiSnEsS", "mInDbUiSnEsS");
    SmartDashboard.putData("Good or Bad? To be or Not to Be?", goodbad);

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
    shooter.start();
    shooter.setSpeed(0);
    //climber.start();
    // controlPanel.start();
    hopper.start();
    intake.start();
    // blinkinLED.start()
    visionManager.start();
    jetsonUDP.start();
    robotTracker.start();
    drive.start();
//c349e488ce1535e2388178b24ceb2496f2a1bdd1


    jetsonUDP.changeExp(true);
  }

  

  @Override
  public void autonomousInit() {
    
    startAll();
    shooter.setSpeed(0);

    robotTracker.resetOdometry();
/*
    autoDone = false;

    int autoDir = 1;
    double startX = 94.6;



    if(dir_chooser.getSelected().equals("Right")) autoDir = -1;
    else autoDir = 1;

    //if(start_chooser.getSelected().equals("Lvl2")) startPos = 18+19-3;//-8;

   // if(m_chooser.getSelected().equals("Cargo 1_2")&& red_blue.getSelected().equals("Red")) option = new Ship1_2Red(autoDir, startPos);
    switch(autoPosition){
      case MIDDLE:
        startX = 94.6;
        break;
      case LEFT:
        startX = 94.6-48;
        break;
      case RIGHT:
        startX = 94.6+48;
        break;
        
    }

    option = new ShootAndMove(startX);
    
    auto = new Thread(option);
    auto.start();
    */
    drive.setRotationHysteresis(Rotation2D.fromDegrees(30));

    
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
    }*//*
    if(option.isFinished()) {
      teleopPeriodic();
    }
    
  
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
    */
  }

  @Override 
  public void teleopInit() {
    startAll();
    
    //killAuto();
    System.out.println("teleop init!");
    //drive.stopMovement();

    //elevator.resetDT();
    //elevator.setHeight(Constants.HatchElevLow);
   // turret.homeTurret();
    //elevator.elevHome();
    //manipulator.setManipulatorIntakeState(Manipulator.ManipulatorIntakeState.OFF);
    firstTeleopRun = true;
    //drive.setTeleop();
    
    
    
  }
  

  int c = 0;
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {


      // System.out.println("intake current: " + intake.getCurrent());
      //System.out.println(drive.getGyroAngle());
     
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

      if(xbox.getRawAxis(2) > 0.5){
        visionManager.setState(VisionStatus.WIN);
      }
      
      else{
        visionManager.setState(VisionStatus.IDLE);
        drive.cheesyDrive(-xbox.getRawAxis(1),  xbox.getRawAxis(4),true);
        if (stick.getRawButton(1)){ //3, 0.5
          //fire shooter
          shooter.setFiring(true);
          hopperOn = true;
  
   
        } else{
          shooter.setFiring(false);
        }
      }
      


      /* x:flywheel togle ✔
      right trigger:shoot  ✔
      button panle: 3 setpoint for shooter hood position + flywheel speed saved to var ✔
      B:Intake in/out, toggle ✔
      left Trigger: Intake stuff  ✔
      left bumber: intake eject ✔
      //Button box: Reverse everything (car wash + intake) ✔

      */

      if (xbox.getRisingEdge(2)){
        intakeSetDeployed = !intakeSetDeployed;

        intake.setDeployState(intakeSetDeployed ? DeployState.DEPLOY:DeployState.UNDEPLOY);

      }

     /* if(xbox.getRisingEdge(3) || buttonPanel.getRisingEdge(5)){
        shooterSetOn=!shooterSetOn;

      }*/

      if(buttonPanel.getRawButton(5)) shooterSetOn = true; 
      else shooterSetOn = false; 

      //shooter presets
      if (buttonPanel.getRisingEdge(1)){
        hoodPosition = 25; //TODO: Adjust numbers
        shooterSpeed = 5500;
      } else if (buttonPanel.getRisingEdge(2)){
        hoodPosition = 33; //TODO: Adjust numbers
        //shooterSpeed = 5000;
        shooterSpeed = 5700;
      } else if (buttonPanel.getRisingEdge(3)){
        hoodPosition = 65; //TODO: Adjust numbers
        shooterSpeed = 3250;
      }

      fireShooter = false;
      ejectShooter = false;
      intakeOn = false;
      ejectAll = false;
      intakeEject = false;
      shooterOn = false;
      hopperEject = false;
      hopperOn = false;

      if (shooterSetOn){
        //Shoter button pushed
        shooterOn = true;
        
      }

      
      /*else {
        fireShooter = false;
        hopperOn = false;
      }*/

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
        shooterOn = false;

        intakeEject = true;
        hopperEject = true;
        ejectShooter = true;


      }

      if(visionManager.getState() == VisionManager.VisionStatus.IDLE) {
      DeployState intakeDeployState = intake.getDeployState();
      if (true || DeployState.DEPLOY == intakeDeployState){
        if(hopperOn){
          hopper.setFrontMotorState(FrontMotorState.ACTIVE);
          hopper.setSnailMotorState(SnailMotorState.ACTIVE);
  
        } else if(hopperEject){
          hopper.setFrontMotorState(FrontMotorState.REVERSE);
          hopper.setSnailMotorState(SnailMotorState.REVERSE);
  
        } else {
          hopper.setFrontMotorState(FrontMotorState.INACTIVE);
          hopper.setSnailMotorState(SnailMotorState.INACTIVE);
  
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
        hopper.setSnailMotorState(SnailMotorState.INACTIVE);

      }

      if (shooterOn){
        shooter.setSpeed(shooterSpeed);
        shooter.setHoodAngle(hoodPosition);
      } else if (!xbox.getRawButton(4)){
        shooter.setSpeed(0);
        //shooter.setFiring(false);
      }
    }

    
    //  shooter.setFiring(fireShooter);
      //maxl = Math.max(Math.abs(drive.getLeftSpeed()), maxl);
      //maxr = Math.max(Math.abs(drive.getRightSpeed()), maxr);

      //System.out.println("velocity L: " +  maxl + " Velocity R: " + maxr);



      /*
      boolean prevShooterOn;
      boolean prevIntakeDeployed;
      double prevHoodPosition;
      int prevShooterSpeed;
      boolean prevEjectAll;
      boolean prevFireShooter;
      boolean prevIntakeOn;
      boolean prevIntakeEject;
      */







     
      

//  ------------------------------------------------------------      
      // if (buttonPanel.getRisingEdge(10)){
      //   ejectAll = true;
      //   hopper.setFrontMotorState(FrontMotorState.REVERSE);
      //   hopper.setSnailMotorState(SnailMotorState.REVERSE);
      //   shooter.setEject(true);

      // }

      // if (buttonPanel.getFallingEdge(10)){
      //   ejectAll = false;
      //   hopper.setFrontMotorState(FrontMotorState.INACTIVE);
      //   hopper.setSnailMotorState(SnailMotorState.INACTIVE);
      //   shooter.setEject(false);
        
      // }


      // if (xbox.getRisingEdge(3)){
      //   shooterSetOn=!shooterSetOn;
        
      //   if(shooterSetOn){
      //     shooter.setSpeed(shooterSpeed);
      //   } else {
      //     shooter.setSpeed(0);
      //   }

      // } //2&3

      // if (xbox.getRisingEdge(3, 0.5)){
      //   shooter.setFiring(true);

      // } 
      // if(xbox.getFallingEdge(3, 0.5)) {
      //   shooter.setFiring(false);

      // }

      // if (xbox.getRisingEdge(2)){
      //   intakeSetDeployed = !intakeSetDeployed;

      //   if(intakeSetDeployed){
      //     intake.setDeployState(DeployState.DEPLOY);
      //   } else{
      //     intake.setDeployState(DeployState.UNDEPLOY);
      //   }

      // }
      // if (intakeSetDeployed){
      //   if (xbox.getRawAxis(2)>0.5 && !ejectAll){
      //     intake.setIntakeState(IntakeState.INTAKE);
      //     hopper.setFrontMotorState(FrontMotorState.ACTIVE);

      //   } else if (xbox.getRisingEdge(5)|| ejectAll){
      //     intake.setIntakeState(IntakeState.EJECT);

      //   } else{
      //     intake.setIntakeState(IntakeState.OFF);

      //   }
      // } else{
      //   intake.setIntakeState(IntakeState.OFF);

      // }



      // if (buttonPanel.getRisingEdge(1)){
      //   hoodPosition = 0; 
      //   shooterSpeed = 2000;
      // } else if (buttonPanel.getRisingEdge(1)){
      //   hoodPosition = 45;
      //   shooterSpeed = 4000;
      // } else if (buttonPanel.getRisingEdge(2)){
      //   hoodPosition = 90;
      //   shooterSpeed = 6000;
      // }





      

  }

  @Override
  public void testInit() {
   // drive.stopMovement();
   // scheduler.resume();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  
  }

  @Override
  public void disabledInit() {
    //killAuto();
   // drive.pause();


    
    scheduler.pause();
    shooter.pause();
    //climber.pause();
    // controlPanel.pause();
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
