package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.command.SendableScript;


@JsonIgnoreProperties(ignoreUnknown = true)
public class ScriptAutonomousStep extends AbstractAutonomousStep {

    private final SendableScript script;

    @JsonCreator
    public ScriptAutonomousStep(@JsonProperty(required = true, value = "script") SendableScript script) {
        this.script = script;
    }

    @Override
    public String toString() {
        return "ScriptAutonomousStep{" + "script='" + script + '\'' + '}';
    }

    @JsonProperty
    public SendableScript getScript() {
        return script;
    }

    @Override
    public void execute(TemplateAuto templateAuto) {
        if(!templateAuto.isDead()){ //Check that our auto is still running
            if (!script.execute()) {
                //The script failed to execute; kill the auto
                templateAuto.killSwitch();
            }
        }
    }
}
