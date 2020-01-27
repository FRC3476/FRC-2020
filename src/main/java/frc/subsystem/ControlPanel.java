package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyCANSparkMax;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.control.RateLimiter;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.*;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




public class ControlPanel extends Subsystem{


    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.colorSensorPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    int wheelPosition = 0;
    int wheelRotation = 0;

    private static LazyCANSparkMax  spinner;



    public enum SpinnerState {
		OFF, SPINNING, DONE
    }
    
    public SpinnerState spinnerState;

    ControlPanel(int period) {
        super(period);

        m_colorMatcher.addColorMatch(Constants.kBlueTarget);
        m_colorMatcher.addColorMatch(Constants.kGreenTarget);
        m_colorMatcher.addColorMatch(Constants.kRedTarget);
        m_colorMatcher.addColorMatch(Constants.kYellowTarget);   
        m_colorMatcher.addColorMatch(Constants.kWhiteTarget);

        spinner = new LazyCANSparkMax(Constants.wheelSpinnerId, MotorType.kBrushless);
        spinner.setIdleMode(IdleMode.kBrake);
        
        spinnerState = SpinnerState.OFF;


        
    }
    //instance = new ControlPanel();

	public SpinnerState getSpinnerState() {
		return spinnerState;
	}


    public void MoveArmDown() {


    }

    public void MoveArmUp() {


    }

    

    public void SpinWheel(){


        
    }






    public void Update(){

        switch(spinnerState){ 
            case SPINNING:
                spinner.set(1.0);

                Color detectedColor = m_colorSensor.getColor();
                char colorString;
                ColorMatchResult match  = m_colorMatcher.matchClosestColor(detectedColor);

                //get Color Looking at
                if (match.color == Constants.kBlueTarget) {
                    colorString = 'B';
                } else if (match.color == Constants.kRedTarget) {
                    colorString = 'R';
                } else if (match.color == Constants.kGreenTarget) {
                    colorString = 'G';
                } else if (match.color == Constants.kYellowTarget) {
                    colorString = 'Y';
                } else {
                    colorString = 'U';
                }

                //check that color is not unknown
                if (colorString!='U'){
            
                    //loop until we find the color we are looking at
                    while (true){
                    
                        //check if we reached the curent color the sensor sees
                        if (Constants.colorWheelOrder[wheelPosition] == colorString){
                            //if we do exit
                            break;
                        }
                    
                    wheelPosition++;

                        //check if we completed a rotation
                        if (wheelPosition == Constants.colorWheelOrder.length ){
                            wheelPosition = 0;
                            wheelRotation++;
                        }
            
                    }

                    if (wheelRotation >= 4){
                        spinnerState = spinnerState.DONE;
                        spinner.set(0.0);

                    }
            
                } 
            
            case OFF:
                spinner.set(0.0);
            
            case DONE:
                spinner.set(0.0);
 

        }




    }












    public void goToColor(){
        
        
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub

    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub

    }

    @Override
    public void logMotorCurrent() {
        // TODO Auto-generated method stub


    }








}
