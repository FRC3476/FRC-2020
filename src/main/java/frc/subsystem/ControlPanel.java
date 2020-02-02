package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyCANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class ControlPanel extends Subsystem{

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.colorSensorPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private char colorString;

    int wheelPosition = 0;
    int wheelRotation = 0;
    int colorConfirmCycle = 0;

    private static LazyCANSparkMax  spinner;

    char feildColorData = 'E';

    private Solenoid spinnerSolenoid;


    public enum SpinnerState {
		OFF, SPINNING, FINDINGCOLOR, CONFIRMINGCOLOR
    }
    
    public SpinnerState spinnerState;


    //getting color data from feild
    private char getFeildColorData(){
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    return 'B';
                case 'G' :
                    return 'G';
                case 'R' :
                    return 'R';
                case 'Y' :
                    return 'Y';
                default :
                    //corupt Data
                    return 'E';
            }
        } else {
            //No value recived yet
            return 'E';
        }

    }

    private char getColorSesorData(){

        Color detectedColor = m_colorSensor.getColor();
                char colorData;
                ColorMatchResult match  = m_colorMatcher.matchClosestColor(detectedColor);

                //get Color Looking at
                if (match.color == Constants.kBlueTarget) {
                    colorData = 'B';
                } else if (match.color == Constants.kRedTarget) {
                    colorData = 'R';
                } else if (match.color == Constants.kGreenTarget) {
                    colorData = 'G';
                } else if (match.color == Constants.kYellowTarget) {
                    colorData = 'Y';
                } else {
                    colorData = 'U';
                }

                return colorData;



    }
    


    //init
    ControlPanel(int period) {
        super(period);

        m_colorMatcher.addColorMatch(Constants.kBlueTarget);
        m_colorMatcher.addColorMatch(Constants.kGreenTarget);
        m_colorMatcher.addColorMatch(Constants.kRedTarget);
        m_colorMatcher.addColorMatch(Constants.kYellowTarget);   
        m_colorMatcher.addColorMatch(Constants.kWhiteTarget);

        spinner = new LazyCANSparkMax(Constants.wheelSpinnerId, MotorType.kBrushless);
        spinner.setIdleMode(IdleMode.kBrake);

        spinnerSolenoid = new Solenoid(Constants.spinnerSolenoidID);
        
        spinnerState = SpinnerState.OFF;


        
    }

	public SpinnerState getSpinnerState() {
		return spinnerState;
	}


    public void MoveDown() {
        spinnerSolenoid.set(false);
    }

    public void MoveUp() {
        spinnerSolenoid.set(true);

    }

    

    public void LevelTwoSpin(){
        if( spinnerSolenoid.get()){
            spinnerState = SpinnerState.SPINNING; 
        } else {

            System.out.println("Still Down");
        }
           
    }

    public void LevelThreeSpin(){
        if( spinnerSolenoid.get()){
            feildColorData = getFeildColorData();

            if (feildColorData != 'E'){
                System.out.println(feildColorData);
                spinnerState = SpinnerState.FINDINGCOLOR;

            } else {
                System.out.println("No Data Recived or Corupted Data");
            }

        } else {
            System.out.println("Still Down");
        }
            
        

        
    }






    public void update(){

        switch(spinnerState){ 
            //spin wheel 4 times
            case SPINNING:
                spinner.set(Constants.wheelSpinnerLevelTwoSpeed);

                colorString = getColorSesorData();
                
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
                        spinnerState = spinnerState.OFF;
                        spinner.set(0.0);
                        System.out.println("Control Panel Spun 4 times");

                    }
            
                } 
                break;
            
            //Move to color specified by FMS
            case FINDINGCOLOR:
                colorString = getColorSesorData();

                if (colorString != feildColorData ){
                    spinner.set(Constants.wheelSpinnerLevelThreeSpeed);

                } else {
                    spinner.set(0.0);
                    colorConfirmCycle = 0;
                    spinnerState = SpinnerState.CONFIRMINGCOLOR;


                }
                    
                break;

            //ensure we stay at the correct color
            case CONFIRMINGCOLOR:
                spinner.set(0.0);
                colorConfirmCycle++;

                colorString = getColorSesorData();
                if (colorString != feildColorData){
                    spinnerState = SpinnerState.FINDINGCOLOR;

                }


                if (colorConfirmCycle >= Constants.colorConfirmCycles){

                    spinnerState = SpinnerState.OFF;
                }
                break;

            case OFF:
                spinner.set(0.0);
                break;
        }

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
