package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.auton.TemplateAuto;
import frc.subsystem.Drive;

import java.util.List;

@JsonIgnoreProperties(ignoreUnknown = true)
public class TrajectoryAutonomousStep extends AbstractAutonomousStep {
    private final List<State> states;

    @JsonCreator
    public TrajectoryAutonomousStep(@JsonProperty(required = true, value = "states")  List<Trajectory.State> m_states) {
        this.states = m_states;
    }

    public Trajectory getTrajectory(){
        return new Trajectory(states);
    }

    @Override
    public void execute(TemplateAuto templateAuto) {
        if(!templateAuto.isDead()){
            Drive.getInstance().setAutoPath(getTrajectory());
            while(!Drive.getInstance().isFinished()) if(templateAuto.isDead()) return;
        }
    }

    @Override
    public String toString() {
        return "TrajectoryAutonomousStep{" +
                "m_states=" + states +
                '}';
    }

    @JsonProperty("states")
    public List<Trajectory.State> getStates() {
        return states;
    }
}