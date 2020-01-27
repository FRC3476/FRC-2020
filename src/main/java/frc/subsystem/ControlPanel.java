package frc.subsystem;

import frc.robot.Constants;
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
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;



public class ControlPanel extends Subsystem{

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.colorSensorPort);

    

    private final ColorMatch m_colorMatcher = new ColorMatch();
    



    int wheelPosition = 0;
    int wheelRotation = 0;

    public void init(){

        m_colorMatcher.addColorMatch(Constants.kBlueTarget);
        m_colorMatcher.addColorMatch(Constants.kGreenTarget);
        m_colorMatcher.addColorMatch(Constants.kRedTarget);
        m_colorMatcher.addColorMatch(Constants.kYellowTarget);   
        m_colorMatcher.addColorMatch(Constants.kWhiteTarget);


    }


    public void MoveArmDown() {


    }

    public void MoveArmUp() {


    }

    

    public void SpinWheel(){

        
    }

    






}
