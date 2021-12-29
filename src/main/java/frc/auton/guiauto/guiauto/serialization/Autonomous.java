package frc.auton.guiauto.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import frc.auton.guiauto.serialization.AbstractAutonomousStep;

import java.util.List;


@JsonIgnoreProperties(ignoreUnknown = true)
public class Autonomous {
    private final List<frc.auton.guiauto.serialization.AbstractAutonomousStep> autonomousSteps;

    @JsonCreator
    public Autonomous(@JsonProperty(required = true, value = "autonomousSteps") List<frc.auton.guiauto.serialization.AbstractAutonomousStep> autonomousSteps) {
        this.autonomousSteps = autonomousSteps;
    }

    @JsonProperty
    public List<AbstractAutonomousStep> getAutonomousSteps() {
        return autonomousSteps;
    }

    @Override
    public String toString() {
        return "Autonomous{" +
                "autonomousSteps=" + autonomousSteps +
                '}';
    }
}