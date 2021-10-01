package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;

import frc.auton.TemplateAuto;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME,
        include = JsonTypeInfo.As.PROPERTY,
        property = "type")
@JsonSubTypes({
        @Type(value = TrajectoryAutonomousStep.class, name = "trajectory"),
        @Type(value = ScriptAutonomousStep.class, name = "script"),
})
@JsonIgnoreProperties(ignoreUnknown = true)
public abstract class AbstractAutonomousStep{

    private final boolean closed;

    @JsonCreator
    protected AbstractAutonomousStep(@JsonProperty(required = true, value = "closed") boolean closed) {
        this.closed = closed;
    }

    public abstract void execute(TemplateAuto templateAuto);

    @JsonProperty("closed")
    public boolean isClosed() {
        return closed;
    }
}
