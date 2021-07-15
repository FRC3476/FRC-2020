// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import frc.robot.Constants;
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


	@Deprecated
	/**
	 * 
	 * @param time Does not do anything
	 * @return current rotaion
	 */
	synchronized public Rotation2D getGyroAngle(long time) {
		return Rotation2D.fromWPIRotation2d(swerveDriveOdometry.getPoseMeters().getRotation());
		
	}

	synchronized public RigidTransform2D getOdometry() {
		return new RigidTransform2D(Translation2D.fromWPITranslation2d(swerveDriveOdometry.getPoseMeters().getTranslation()), 
			Rotation2D.fromWPIRotation2d(swerveDriveOdometry.getPoseMeters().getRotation()));
	}

	/**
	 * Resets the position on the feild to 0,0 with a rotaion of 0 degrees
	 */
	synchronized public void resetOdometry() {
		swerveDriveOdometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(drive.getAngle()));
	}

  	/**
   	* Updates the robot's position on the field using forward kinematics and integration of the pose
   	* over time. This method automatically calculates the current time to calculate period
   	* (difference between two timestamps). The period is used to calculate the change in distance
   	* from a velocity. This also takes in an angle parameter which is used instead of the angular
   	* rate that is calculated from forward kinematics.
   	*
   	* @param gyroAngle The angle reported by the gyroscope.
   	* @param moduleStates The current state of all swerve modules. Please provide the states in the
   	*     same order in which you instantiated your SwerveDriveKinematics.
   	* @return The new pose of the robot.
   	*/
	@Override
	public void update() {
		swerveDriveOdometry.update(Rotation2d.fromDegrees(drive.getAngle()), drive.getSwerveModuleStates());
	}

	/**
	 * You should not use this. Use {@link RobotTracker#resetPosition​} instead
	 * @param offset 
	 */
	@Deprecated
	synchronized public void setInitialRotation(Rotation2D offset) {
		swerveDriveOdometry.resetPosition(new Pose2d(swerveDriveOdometry.getPoseMeters().getTranslation(), offset.getWPIRotation2d()), Rotation2d.fromDegrees(drive.getAngle()));
	}

	
	/**
	 * You should not use this. Use {@link RobotTracker#resetPosition​} instead
	 * @param offset 
	 */
	@Deprecated
	synchronized public void setInitialTranslation(Translation2D offset) {
		
		swerveDriveOdometry.resetPosition(new Pose2d(offset.getWPITranslation2d(), swerveDriveOdometry.getPoseMeters().getRotation()), Rotation2d.fromDegrees(drive.getAngle()));
	}

	/**
	 * Resets the robot's position on the field.
	 * The gyroscope angle does not need to be reset here on the user's robot code. The library automatically takes care of offsetting the gyro angle.
	 * 
	 * @param pose The position on the field that your robot is at.
	 * @param gyroAngle The position on the field that your robot is at.
	 */
	synchronized public void resetPosition(Pose2d pose, Rotation2d gyroAngle){
		swerveDriveOdometry.resetPosition(pose, gyroAngle);
	}

	/**
   	* Returns the position of the robot on the field.
   	*
   	* @return The pose of the robot (x and y are in meters).
   	*/
	synchronized public Pose2d getPoseMeters(){
		return swerveDriveOdometry.getPoseMeters();
	}

	@Override
	public void selfTest() {

	}

	@Override
	public void logData() {

	}
}