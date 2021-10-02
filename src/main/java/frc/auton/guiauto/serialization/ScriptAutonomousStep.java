package frc.auton.guiauto.serialization;

import java.lang.invoke.WrongMethodTypeException;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

import frc.auton.TemplateAuto;
import frc.subsystem.Hopper;
import frc.subsystem.Intake;
import frc.subsystem.Shooter;
import frc.subsystem.VisionManager;

@JsonIgnoreProperties(ignoreUnknown = true)
public class ScriptAutonomousStep extends AbstractAutonomousStep {

    private final String script;
    private final boolean valid;

    ScriptEngineManager manager;
    ScriptEngine engine;
    {
        manager = new ScriptEngineManager();
        engine = manager.getEngineByName("JavaScript");

        engine.put("shooter", Shooter.getInstance());
        engine.put("hopper", Hopper.getInstance());
        engine.put("intake", Intake.getInstance());
        engine.put("visionManager", VisionManager.getInstance());
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
                engine.put("auto", templateAuto);
                engine.eval(script);
            } catch (ScriptException | WrongMethodTypeException exception) {
                System.out.println("Script " + script + " Failed to Execute");
            } catch (Exception e){
                System.out.println("Unexpected Exception executing "  + script);
                e.printStackTrace();
                templateAuto.killSwitch();
            }
        }


    }
}
