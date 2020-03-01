package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyCANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ControlPanel extends Subsystem {

    private final ColorSensorV3 colorSensor = new ColorSensorV3(Constants.colorSensorPort);
    private final ColorMatch colorMatcher = new ColorMatch();
    private char colorString;

    int wheelPosition = 0;
    int wheelRotation = 0;
    int colorConfirmCycle = 0;

    private static LazyCANSparkMax spinner;


    char feildColorData = 'E';
    char usablefeildColorData;

    private Solenoid spinnerSolenoid;

    public enum SpinnerState {
        OFF, SPINNING, FINDINGCOLOR, CONFIRMINGCOLOR
    }

    public SpinnerState spinnerState;

    // getting color data from feild
    private char getFeildColorData() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                return 'B';
            case 'G':
                return 'G';
            case 'R':
                return 'R';
            case 'Y':
                return 'Y';
            default:
                // corupt Data
                return 'E';
            }
        } else {
            // No value recived yet
            return 'E';
        }

    }

    public char getColorSensorData() {

        Color detectedColor = colorSensor.getColor();
        char colorData;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        // get Color Looking at
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

//-------------------------------------------------------------


    private static final ControlPanel instance = null;//new ControlPanel(); 
    public static ControlPanel getInstance() {
        return instance;
    }

    // init
    ControlPanel() {
        super(Constants.controlPanelPeriod);

        colorMatcher.addColorMatch(Constants.kBlueTarget);
        colorMatcher.addColorMatch(Constants.kGreenTarget);
        colorMatcher.addColorMatch(Constants.kRedTarget);
        colorMatcher.addColorMatch(Constants.kYellowTarget);
        colorMatcher.addColorMatch(Constants.kWhiteTarget);

        spinner = new LazyCANSparkMax(Constants.wheelSpinnerId, MotorType.kBrushless);


        spinner.setIdleMode(IdleMode.kBrake);

        spinnerSolenoid = new Solenoid(Constants.spinnerSolenoidID);
        spinnerSolenoid.set(false);

        spinnerState = SpinnerState.OFF;

    }

    public synchronized SpinnerState getSpinnerState() {
        return spinnerState;
    }

    public synchronized void deploy() {
        spinnerSolenoid.set(false);
    }

    public synchronized void unDeploy() {
        spinnerSolenoid.set(true);

    }

    public synchronized void doLevelTwoSpin() {
        wheelPosition = 0;
        wheelRotation = 0;
        spinnerState = SpinnerState.SPINNING;

    }

    public synchronized void doLevelThreeSpin() {
        feildColorData = getFeildColorData();

        if (feildColorData != 'E') {
            
            int pos = -1;
            for(int i = 0; i < Constants.colorWheelOrder.length; i++) {
                if(Constants.colorWheelOrder[i] == feildColorData) {
                    pos = i;
                    break;
                }
            }

            usablefeildColorData = Constants.colorWheelOrder[pos+Constants.LevelThreeColorOffset];

            System.out.println("Feild Color Data: " + feildColorData);
            System.out.println("Using Color: " + usablefeildColorData);
            spinnerState = SpinnerState.FINDINGCOLOR;

        } else {
            System.out.println("No Data Recived or Corupted Data");
        }

    }



    public synchronized void stopSpin() {
        spinnerState = SpinnerState.OFF;
        spinner.set(0);
    }

// --------------------------------------------------------------

    public synchronized void update() {

        switch (spinnerState) {
        // spin wheel 4 times
        case SPINNING:
            spinner.set(Constants.wheelSpinnerLevelTwoSpeed);

            colorString = getColorSensorData();

            // check that color is not unknown
            if (colorString != 'U') {

                // loop until we find the color we are looking at
                while (!(Constants.colorWheelOrder[wheelPosition] == colorString)) {

                    wheelPosition++;

                    // check if we completed a rotation
                    if (wheelPosition == Constants.colorWheelOrder.length) {
                        wheelPosition = 0;
                        wheelRotation++;
                    }

                }

                if (wheelRotation >= 4) {
                    spinnerState = SpinnerState.OFF;
                    System.out.println("Control Panel Spun 4 times");

                }

            }
            break;

        case FINDINGCOLOR:
            colorString = getColorSensorData();

            if (colorString != usablefeildColorData) {
                spinner.set(Constants.wheelSpinnerLevelThreeSpeed);

            } else {
                colorConfirmCycle = 0;
                spinnerState = SpinnerState.CONFIRMINGCOLOR;

            }

            break;

        // ensure we stay at the correct color
        case CONFIRMINGCOLOR:
            spinner.set(0.0);
            colorConfirmCycle++;

            colorString = getColorSensorData();
            if (colorString != usablefeildColorData) {
                spinnerState = SpinnerState.FINDINGCOLOR;

            }

            if (colorConfirmCycle >= Constants.colorConfirmCycles) {

                spinnerState = SpinnerState.OFF;
            }
            break;

        case OFF:
            spinner.set(0.0);
            break;
        }

    }

//----------------------------------------------------------

    @Override
    public void selfTest() {
        spinner.set(1.0);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            System.out.println("Sleep Failed" + e);
        }
        spinner.set(0.0);

        spinnerSolenoid.set(true);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            System.out.println("Sleep Failed" + e);
        }
        spinnerSolenoid.set(false);

        double TargetTime = Timer.getFPGATimestamp()+8;

        while (Timer.getFPGATimestamp() < TargetTime){



        }

    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub

    }


    public void logMotorCurrent() {
        // TODO Auto-generated method stub
    }
}
