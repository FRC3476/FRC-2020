// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.math;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Stores a Translation2D and a Rotation2D
 */
public class RigidTransform2D implements Interpolable<RigidTransform2D> {

	public Rotation2D rotationMat;
	public Translation2D translationMat;

	public RigidTransform2D() {
		rotationMat = new Rotation2D();
		translationMat = new Translation2D();
	}

	public RigidTransform2D(Translation2D translation, Rotation2D rotation) {
		rotationMat = rotation;
		translationMat = translation;
	}

	@Override
	public RigidTransform2D interpolate(RigidTransform2D other, double percentage) {
		return new RigidTransform2D(this.translationMat.interpolate(other.translationMat, percentage),
				this.rotationMat.interpolate(other.rotationMat, percentage));
	}

	/**
	 * Translates delta rotated by our rotation matrix and rotates our rotation
	 * matrix by the other rotation matrix
	 *
	 * @param delta
	 *
	 * @return
	 */
	public RigidTransform2D transform(RigidTransform2D delta) {
		return new RigidTransform2D(translationMat.translateBy(delta.translationMat.rotateBy(rotationMat)),
				rotationMat.rotateBy(delta.rotationMat));
	}

	public static RigidTransform2D fromWPIPose2d(Pose2d pose2d) {
		return new RigidTransform2D(Translation2D.fromWPITranslation2d(pose2d.getTranslation()), Rotation2D.fromWPIRotation2d(pose2d.getRotation()));
	}
}
