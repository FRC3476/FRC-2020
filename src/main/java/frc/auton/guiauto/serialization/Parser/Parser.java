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
        String[] commands = string.split("\n");
        for (String command : commands) {
            StringBuilder methodName = null;
            StringBuilder argument = null;
            for (int i = 0; i < command.length(); i++) {
                char character = command.charAt(i);
                if(argument != null){
                    argument.append(character);
                } else if(methodName == null){
                    if(!Character.isWhitespace(character)){
                        methodName = new StringBuilder().append(character);
                    }

                } else if(!Character.isWhitespace(character)){
                    methodName.append(character);

                } else {
                    argument = new StringBuilder();
                }
            }

            //System.out.println("Command " + command + " method name: " + methodName + " argument: " + argument);
            if(methodName == null) return false;
            switch (methodName.toString()){
                case "print":
                    if(argument == null) return false;
                    System.out.println(argument);
                    break;
                case "shootBalls":
                    try{
                        context.shootBallsTimed(Float.parseFloat(argument.toString()));
                    } catch (NumberFormatException | NullPointerException e) { return false; } 
                    break;
                case "setShooterSpeed":
                    try{
                        Shooter.getInstance().setSpeed(Float.parseFloat(argument.toString()));
                    } catch (NumberFormatException | NullPointerException e) { return false; } 
                    break;
                case "sleep":
                    try{
                        OrangeUtility.sleep(Long.parseLong(argument.toString()));
                    } catch (NumberFormatException | NullPointerException e) { return false; } 
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
        }
        return true;
    }
}

