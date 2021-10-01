package frc.auton.guiauto.serialization;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class Autonomous{
    private final List<AbstractAutonomousStep> autonomousSteps;

    @JsonCreator
    public Autonomous(@JsonProperty(required = true, value = "autonomousSteps") List<AbstractAutonomousStep> autonomousSteps){

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
