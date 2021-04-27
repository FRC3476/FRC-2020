// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import frc.robot.Constants;
import frc.utility.CircularQueue;
import frc.utility.math.InterpolablePair;
import frc.utility.math.RigidTransform2D;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class RobotTracker extends Subsystem {

	private static final RobotTracker trackingInstance = new RobotTracker();
	double startTime;

	Drive drive = Drive.getInstance();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	private SwerveDriveOdometry swerveDriveOdometry;

	private RobotTracker() {
		super(Constants.RobotTrackerPeriod);
		swerveDriveOdometry = new SwerveDriveOdometry(drive.swerveKinematics, drive.getGyroAngle().getWPIRotation2d());
		startTime = Timer.getFPGATimestamp();
	}

	synchronized public Rotation2D getGyroAngle(long time) {
		return null;
		
	}

	synchronized public RigidTransform2D getOdometry() {
		return null;
	}

	synchronized public void resetOdometry() {
		
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed
	 */
	@Override
	public void update() {
		
	}

	/**
	 * You should not use this. Use {@link RobotTracker#resetPosition​} instead
	 * @param offset 
	 */
	@Deprecated
	synchronized public void setInitialRotation(Rotation2D offset) {
		
	}

	
	/**
	 * You should not use this. Use {@link RobotTracker#resetPosition​} instead
	 * @param offset 
	 */
	@Deprecated
	synchronized public void setInitialTranslation(Translation2D offset) {
		
		resetOdometry();
	}

	/**
	 * Resets the robot's position on the field.
	 * The gyroscope angle does not need to be reset here on the user's robot code. The library automatically takes care of offsetting the gyro angle.
	 * 
	 * @param pose The position on the field that your robot is at.
	 * @param gyroAngle The position on the field that your robot is at.
	 */
	synchronized public void resetPosition​(Pose2d pose, Rotation2d gyroAngle){
		swerveDriveOdometry.resetPosition(pose, gyroAngle);
	}


	synchronized public Pose2d getPoseMeters(){
		return null;
	}

	@Override
	public void selfTest() {

	}

	@Override
	public void logData() {

	}
}