package frc.auton.guiauto.serialization.Parser;

import frc.auton.TemplateAuto;
import frc.subsystem.Hopper;
import frc.subsystem.Intake;
import frc.subsystem.Shooter;
import frc.subsystem.VisionManager;
import frc.subsystem.Hopper.FrontMotorState;
import frc.subsystem.Hopper.SnailMotorState;
import frc.subsystem.Intake.DeployState;
import frc.subsystem.Intake.IntakeState;
import frc.subsystem.VisionManager.VisionStatus;
import frc.utility.OrangeUtility;

public class Parser {
    public static boolean execute(String string, TemplateAuto context){
        String[] commands = string.split("\n"); //Start by spliting everything by lines (commands)
        for (String command : commands) {
            StringBuilder methodName = null;
            StringBuilder argument = null;
            for (int i = 0; i < command.length(); i++) { //Loop though all the characters
                char character = command.charAt(i);
                if(argument != null){
                    //We've reached the argument part of the command. Everything else is part of the argument
                    argument.append(character);
                } else if(methodName == null){
                    //We haven't initalized the beginning of the methodName part of the command yet
                    //This means we haven't seen the first character of the method name yet
                    if(!Character.isWhitespace(character)){
                        //If we see the first character initalize the method name add add the character
                        methodName = new StringBuilder().append(character);
                    } //else do nothing and continue looping until we find our first character
                } else if(!Character.isWhitespace(character)){
                    // Keep adding to the method name until we hit whitespace
                    methodName.append(character);
                } else {
                    //We've hit section of whitespace directly before the argument part of the command
                    //Initalize the argument to record that we've reached this section and skip the first whitespace
                    argument = new StringBuilder();
                }
            }

            //System.out.println("Command " + command + " method name: " + methodName + " argument: " + argument);
            if(methodName != null){ //Check that we don't have a blank command/methodName
                try{    
                    //Figure out what the command is and execute it
                    switch (methodName.toString()){
                        case "print":
                            System.out.println(argument);
                            break;
                        case "shootBalls":
                                context.shootBallsTimed(Float.parseFloat(argument.toString()));
                            break;
                        case "setShooterSpeed":
                                Shooter.getInstance().setSpeed(Float.parseFloat(argument.toString()));
                            break;
                        case "sleep":
                                OrangeUtility.sleep(Long.parseLong(argument.toString()));
                            break;
                        case "deployIntake":
                            Intake.getInstance().setDeployState(DeployState.DEPLOY);
                            break;
                        case "undeployIntake":
                            Intake.getInstance().setDeployState(DeployState.UNDEPLOY);
                            break;
                        case "intakeOn":
                            Intake.getInstance().setIntakeState(IntakeState.INTAKE);
                            break;
                        case "intakeOff":
                            Intake.getInstance().setIntakeState(IntakeState.OFF);
                            break;
                        case "intakeReverse":
                            Intake.getInstance().setIntakeState(IntakeState.EJECT);
                            break;
                        case "snailOn":
                            Hopper.getInstance().setSnailMotorState(SnailMotorState.ACTIVE, false);
                            break;
                        case "snailOff":
                            Hopper.getInstance().setSnailMotorState(SnailMotorState.INACTIVE, false);
                            break;
                        case "snailReverse":
                            Hopper.getInstance().setSnailMotorState(SnailMotorState.REVERSE, false);
                            break;
                        case "frontActive":
                            Hopper.getInstance().setFrontMotorState(FrontMotorState.ACTIVE);
                            break;
                        case "frontInactive":
                            Hopper.getInstance().setFrontMotorState(FrontMotorState.INACTIVE);
                            break;
                        case "frontReverse":
                            Hopper.getInstance().setFrontMotorState(FrontMotorState.REVERSE);
                            break;
                        case "visionIdle":
                            VisionManager.getInstance().setState(VisionStatus.IDLE);
                            break;
                        case "visionWin":
                            VisionManager.getInstance().setState(VisionStatus.WIN);
                            break;
                        case "visionAim":
                            VisionManager.getInstance().setState(VisionStatus.AIMING);
                            break;
                        case "fireShooter":
                            Shooter.getInstance().setFiring(true);
                            break;
                        case "stopFiringShooter":
                            Shooter.getInstance().setFiring(false);
                            break;
                        case "turnOnIntakeTrack":
                            context.turnOnIntakeTrack();
                            break;
                        case "turnOffIntakeTrack":
                            context.turnOffIntakeTrack();
                            break;
                        default:
                            return false; 
                    }
                } catch (NumberFormatException | NullPointerException e) { return false; } 
            } //else and continue to the next command (or auto step)
            
        }
        return true;
    }
}

