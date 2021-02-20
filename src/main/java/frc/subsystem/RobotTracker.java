// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.*;
import frc.robot.Constants;
import frc.utility.math.RigidTransform2D;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class RobotTracker extends Subsystem {

	private static final RobotTracker trackingInstance = new RobotTracker();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	DifferentialDrivePoseEstimator differentialDrivePoseEstimator;	
	Drive driveBase;
	double leftPrevDistInches;
	double rightPrevDistInches;

	private RobotTracker() {
		super(Constants.RobotTrackerPeriod);
		driveBase = Drive.getInstance();
		leftPrevDistInches = driveBase.getLeftDistance();
		rightPrevDistInches = driveBase.getRightDistance();
		differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(driveBase.getAngle()), new Pose2d(),
        new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
		new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01), // Global measurement standard deviations. X, Y, and theta.
		Constants.RobotTrackerPeriod/1000); 
		
	}

	synchronized public Rotation2D getGyroAngle(long time) {
		return null;
	}

	synchronized public RigidTransform2D getOdometry() {
		Pose2d pose = differentialDrivePoseEstimator.getEstimatedPosition();
		return new RigidTransform2D(Translation2D.fromWPITranslation2d(pose.getTranslation()), Rotation2D.fromWPIRotation2d(pose.getRotation()));
	}

	synchronized public void resetOdometry() {
		differentialDrivePoseEstimator.resetPosition(new Pose2d(), new Rotation2d(driveBase.getAngle()));
		leftPrevDistInches = driveBase.getLeftDistance();
		rightPrevDistInches = driveBase.getRightDistance();
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed
	 */
	@Override
	public void update() {

		differentialDrivePoseEstimator.update(new Rotation2d(driveBase.getAngle()), new DifferentialDriveWheelSpeeds(driveBase.getLeftSpeed()*Constants.InchesPerMeter, 
		driveBase.getRightSpeed()*Constants.InchesPerMeter), 
		(driveBase.getLeftDistance()-leftPrevDistInches)*Constants.InchesPerMeter, (driveBase.getRightDistance()-rightPrevDistInches)*Constants.InchesPerMeter);
	}

	/**
	 *
	 * 
	 */
	synchronized public void setInitialRotation(Rotation2D rotation2D) {
		Pose2d pose = differentialDrivePoseEstimator.getEstimatedPosition();
		differentialDrivePoseEstimator.resetPosition(new Pose2d(pose.getTranslation(), rotation2D.getWPIRotation2d()), new Rotation2d(driveBase.getAngle()));
		leftPrevDistInches = driveBase.getLeftDistance();
		rightPrevDistInches = driveBase.getRightDistance();
	}

	synchronized public void setInitialTranslation(Translation2D translation2D) {
		Pose2d pose = differentialDrivePoseEstimator.getEstimatedPosition();
		differentialDrivePoseEstimator.resetPosition(new Pose2d(translation2D.getWPITranslation2d(), pose.getRotation()), new Rotation2d(driveBase.getAngle()));
		leftPrevDistInches = driveBase.getLeftDistance();
		rightPrevDistInches = driveBase.getRightDistance();
	}
	/**
	 * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise.
	 * This method can be called as infrequently as you want
	 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds The timestamp of the vision measurement in seconds.
	 */
	synchronized public void addVisionMeasurment(Pose2d visionRobotPoseMeters, double timestampSeconds){
		differentialDrivePoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
	}

	@Override
	public void selfTest() {

	}

	@Override
	public void logData() {

	}
}