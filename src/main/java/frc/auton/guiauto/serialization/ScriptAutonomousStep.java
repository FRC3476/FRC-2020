package frc.auton.guiauto.serialization;

import java.lang.invoke.WrongMethodTypeException;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.wpilibj.DriverStation;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.Auto;
import frc.subsystem.Hopper;
import frc.subsystem.Intake;
import frc.subsystem.Shooter;
import frc.subsystem.VisionManager;

@JsonIgnoreProperties(ignoreUnknown = true)
public class ScriptAutonomousStep extends AbstractAutonomousStep {

    private final String script;
    private final boolean valid;

    static ScriptEngineManager manager;
    static ScriptEngine engine;

    public static void init(){
        manager = new ScriptEngineManager();
        engine = manager.getEngineByName("JavaScript");

        engine.put("shooter", Shooter.getInstance());
        engine.put("hopper", Hopper.getInstance());
        engine.put("intake", Intake.getInstance());
        engine.put("visionManager", VisionManager.getInstance());
        engine.put("auto", Auto.getInstance());

        try {
            engine.eval("print(\"I'm\")");
            engine.eval("print(\"just\")");
            engine.eval("print(\"warming\")");
            engine.eval("print(\"up\")");
            engine.eval("print(\"you\")");
            engine.eval("print(\"can\")");
            engine.eval("print(\"ignore\")");
            engine.eval("print(\"this\")");
            engine.eval("print(\":)\")");
            engine.eval("intake.setDeployStateDeploy()");
            engine.eval("intake.setDeployStateUnDeploy();");
            engine.eval("intake.setIntakeStateIntake()");
            engine.eval("intake.setIntakeStateEject()");
            engine.eval("intake.setIntakeStateOff();\nhopper.setSnailMotorStateActive(false)");
            engine.eval("hopper.setSnailMotorStateReverse(true);\nhopper.setSnailMotorStateInactive(false)");
            engine.eval("hopper.setFrontMotorStateActive();\nhopper.setFrontMotorStateInactive()");
            engine.eval("visionManager.setStateAiming()");
            engine.eval("visionManager.setStateWin()");
            engine.eval("visionManager.setStateIdle()");
            engine.eval("shooter.setSpeed(5000)\nshooter.setFiring(true)\nshooter.setSpeed(0)\nshooter.setFiring(false)");

            engine.eval("print(\"I'm\")");
            engine.eval("print(\"just\")");
            engine.eval("print(\"warming\")");
            engine.eval("print(\"up\")");
            engine.eval("print(\"you\")");
            engine.eval("print(\"can\")");
            engine.eval("print(\"ignore\")");
            engine.eval("print(\"this\")");
            engine.eval("print(\":)\")");
            engine.eval("intake.setDeployStateDeploy()");
            engine.eval("intake.setDeployStateUnDeploy();");
            engine.eval("intake.setIntakeStateIntake()");
            engine.eval("intake.setIntakeStateEject()");
            engine.eval("intake.setIntakeStateOff();\nhopper.setSnailMotorStateActive(false)");
            engine.eval("hopper.setSnailMotorStateReverse(true);\nhopper.setSnailMotorStateInactive(false)");
            engine.eval("hopper.setFrontMotorStateActive();\nhopper.setFrontMotorStateInactive()");
            engine.eval("visionManager.setStateAiming()");
            engine.eval("visionManager.setStateWin()");
            engine.eval("visionManager.setStateIdle()");
            engine.eval("shooter.setSpeed(5000)\nshooter.setFiring(true)\nshooter.setSpeed(0)\nshooter.setFiring(false)");

            engine.eval("print(\"I'm\")");
            engine.eval("print(\"just\")");
            engine.eval("print(\"warming\")");
            engine.eval("print(\"up\")");
            engine.eval("print(\"you\")");
            engine.eval("print(\"can\")");
            engine.eval("print(\"ignore\")");
            engine.eval("print(\"this\")");
            engine.eval("print(\":)\")");
            engine.eval("intake.setDeployStateDeploy()");
            engine.eval("intake.setDeployStateUnDeploy();");
            engine.eval("intake.setIntakeStateIntake()");
            engine.eval("intake.setIntakeStateEject()");
            engine.eval("intake.setIntakeStateOff();\nhopper.setSnailMotorStateActive(false)");
            engine.eval("hopper.setSnailMotorStateReverse(true);\nhopper.setSnailMotorStateInactive(false)");
            engine.eval("hopper.setFrontMotorStateActive();\nhopper.setFrontMotorStateInactive()");
            engine.eval("visionManager.setStateAiming()");
            engine.eval("visionManager.setStateWin()");
            engine.eval("visionManager.setStateIdle()");
            engine.eval("shooter.setSpeed(5000)\nshooter.setFiring(true)\nshooter.setSpeed(0)\nshooter.setFiring(false)");

            engine.eval("print(\"I'm\")");
            engine.eval("print(\"just\")");
            engine.eval("print(\"warming\")");
            engine.eval("print(\"up\")");
            engine.eval("print(\"you\")");
            engine.eval("print(\"can\")");
            engine.eval("print(\"ignore\")");
            engine.eval("print(\"this\")");
            engine.eval("print(\":)\")");
            engine.eval("intake.setDeployStateDeploy()");
            engine.eval("intake.setDeployStateUnDeploy();");
            engine.eval("intake.setIntakeStateIntake()");
            engine.eval("intake.setIntakeStateEject()");
            engine.eval("intake.setIntakeStateOff();\nhopper.setSnailMotorStateActive(false)");
            engine.eval("hopper.setSnailMotorStateReverse(true);\nhopper.setSnailMotorStateInactive(false)");
            engine.eval("hopper.setFrontMotorStateActive();\nhopper.setFrontMotorStateInactive()");
            engine.eval("visionManager.setStateAiming()");
            engine.eval("visionManager.setStateWin()");
            engine.eval("visionManager.setStateIdle()");
            engine.eval("shooter.setSpeed(5000)\nshooter.setFiring(true)\nshooter.setSpeed(0)\nshooter.setFiring(false)");

            engine.eval("print(\"I'm\")");
            engine.eval("print(\"just\")");
            engine.eval("print(\"warming\")");
            engine.eval("print(\"up\")");
            engine.eval("print(\"you\")");
            engine.eval("print(\"can\")");
            engine.eval("print(\"ignore\")");
            engine.eval("print(\"this\")");
            engine.eval("print(\":)\")");
            engine.eval("intake.setDeployStateDeploy()");
            engine.eval("intake.setDeployStateUnDeploy();");
            engine.eval("intake.setIntakeStateIntake()");
            engine.eval("intake.setIntakeStateEject()");
            engine.eval("intake.setIntakeStateOff();\nhopper.setSnailMotorStateActive(false)");
            engine.eval("hopper.setSnailMotorStateReverse(true);\nhopper.setSnailMotorStateInactive(false)");
            engine.eval("hopper.setFrontMotorStateActive();\nhopper.setFrontMotorStateInactive()");
            engine.eval("visionManager.setStateAiming()");
            engine.eval("visionManager.setStateWin()");
            engine.eval("visionManager.setStateIdle()");
            engine.eval("shooter.setSpeed(5000)\nshooter.setFiring(true)\nshooter.setSpeed(0)\nshooter.setFiring(false)");

            engine.eval("print(\"I'm\")");
            engine.eval("print(\"just\")");
            engine.eval("print(\"warming\")");
            engine.eval("print(\"up\")");
            engine.eval("print(\"you\")");
            engine.eval("print(\"can\")");
            engine.eval("print(\"ignore\")");
            engine.eval("print(\"this\")");
            engine.eval("print(\":)\")");
            engine.eval("intake.setDeployStateDeploy()");
            engine.eval("intake.setDeployStateUnDeploy();");
            engine.eval("intake.setIntakeStateIntake()");
            engine.eval("intake.setIntakeStateEject()");
            engine.eval("intake.setIntakeStateOff();\nhopper.setSnailMotorStateActive(false)");
            engine.eval("hopper.setSnailMotorStateReverse(true);\nhopper.setSnailMotorStateInactive(false)");
            engine.eval("hopper.setFrontMotorStateActive();\nhopper.setFrontMotorStateInactive()");
            engine.eval("visionManager.setStateAiming()");
            engine.eval("visionManager.setStateWin()");
            engine.eval("visionManager.setStateIdle()");
            engine.eval("shooter.setSpeed(5000)\nshooter.setFiring(true)\nshooter.setSpeed(0)\nshooter.setFiring(false)");

            

        } catch (ScriptException e) {
            // TODO Auto-generated catch block
            DriverStation.reportError("failed to init script engine", e.getStackTrace());
        }
        
    }

    @JsonCreator
    public ScriptAutonomousStep(@JsonProperty(required = true, value = "script") String script,
            @JsonProperty(required = true, value = "closed") boolean closed,
            @JsonProperty(required = true, value = "valid") boolean valid) {
        super(closed);
        this.script = script;
        this.valid = valid;
    }


    @Override
    public String toString() {
        return "ScriptAutonomousStep{" + "script='" + script + '\'' + '}';
    }

    @JsonProperty
    public String getScript() {
        return script;
    }

    @JsonProperty
    public boolean isValid() {
        return valid;
    }

    @Override
    public void execute(TemplateAuto templateAuto) {
        if(!templateAuto.isDead()){
            try {
                Auto.setContext(templateAuto);
                engine.eval(script);
            } catch (ScriptException | WrongMethodTypeException exception) {
                DriverStation.reportError("Script " + script + " Failed to Execute", exception.getStackTrace());
                templateAuto.killSwitch();
            } catch (Exception e){
                DriverStation.reportError("Unexpected Exception executing "  + script, e.getStackTrace());
                templateAuto.killSwitch();
            }
        }


    }
}
