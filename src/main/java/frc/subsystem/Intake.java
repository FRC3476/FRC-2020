// package frc.subsystem;

// import frc.robot.Constants;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import frc.utility.telemetry.TelemetryServer;
// import frc.utility.LazyTalonSRX;
// import frc.utility.LazySparkSRX;
// import edu.wpi.first.wpilibj.Solenoid;

// public class Intake extends Subsystem {

//     public enum DeployState {
//             DEPLOY, UNDEPLOY, DEPLOYING
//     }
//     public enum IntakeState {
//             ACTIVATED, UNACTIVATED, RELEASE
//     }

// instance = new Intake();

// private TelemetryServer telemetryServer = TelemetryServer.getInstance();
// private Solenoid deploySolenoid;
// private LazyTalonSRX intakeMotor;
// private LazySparkSRX intakeMotor;
// private DeployState deployState = DeployState.UNDEPLOY;
// private DeployState deployState = DeployState.UNACTIVATED;

// private double lastDeployCommandTime;

// private Intake() {
