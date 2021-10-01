package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.auton.TemplateAuto;
import frc.subsystem.Drive;

import java.util.List;

public class TrajectoryAutonomousStep extends AbstractAutonomousStep {
    private final List<State> states;
    private final List<Pose2d> pose2DList;
    private final boolean reversed;
    private final float color;

    Drive drive = Drive.getInstance();

    @JsonCreator
    public TrajectoryAutonomousStep(@JsonProperty(required = true, value = "states")  List<Trajectory.State> m_states,
                                    @JsonProperty(required = true, value = "pointList") List<Pose2d> point2DList,
                                    @JsonProperty(required = true, value = "reversed") boolean reversed,
                                    @JsonProperty(required = true, value = "color") float color,
                                    @JsonProperty(required = true, value = "closed") boolean closed) {
        super(closed);
        this.reversed = reversed;
        this.color = color;
        this.pose2DList = point2DList;
        this.states = m_states;


    }

    public Trajectory getTrajectory(){
        return new Trajectory(states);
    }

    @JsonProperty("pointList")
    public List<Pose2d> getPose2DList(){

        return pose2DList;
    }

    @JsonProperty
    public boolean isReversed(){
        return reversed;
    }


    @Override
    public void execute(TemplateAuto templateAuto) {
        if(!templateAuto.isDead()){
            drive.setAutoPath(getTrajectory());
            while(!drive.isFinished()) if(templateAuto.isDead()) return;
        }
    }

    @Override
    public String toString() {
        return "TrajectoryAutonomousStep{" +
                "m_states=" + states +
                ", pose2DList=" + pose2DList +
                ", reversed=" + reversed +
                ", color=" + color +
                '}';
    }

    @JsonProperty
    public float getColor() {
        return color;
    }

    @JsonProperty("states")
    public List<Trajectory.State> getStates() {
        return states;
    }
}
