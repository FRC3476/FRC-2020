package frc.auton.guiauto.serialization;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.Parser.Parser;


@JsonIgnoreProperties(ignoreUnknown = true)
public class ScriptAutonomousStep extends AbstractAutonomousStep {

    private final String script;

    static ScriptEngineManager manager;
    static ScriptEngine engine;

    @JsonCreator
    public ScriptAutonomousStep(@JsonProperty(required = true, value = "script") String script) {
        this.script = script;
    }


    @Override
    public String toString() {
        return "ScriptAutonomousStep{" + "script='" + script + '\'' + '}';
    }

    @JsonProperty
    public String getScript() {
        return script;
    }

    @Override
    public void execute(TemplateAuto templateAuto) {
        if(!templateAuto.isDead()){
            if(!Parser.execute(this.getScript(), templateAuto)){
                templateAuto.killSwitch();
            }
        }


    }
}