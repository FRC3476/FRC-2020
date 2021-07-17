package frc.auton;

import javax.annotation.Nonnegative;

import frc.utility.control.motion.Path;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public abstract class SimpleAutonomous extends TemplateAuto {
    
    Translation2D lastTranslation2d = new Translation2D();
    Rotation2D lastRotation2d = new Rotation2D();
    double defaultSpeed;

    public SimpleAutonomous(Translation2D startTranslation, Rotation2D startRotation, double defaultSpeed) {
        super(startTranslation);
        lastTranslation2d = startTranslation;
        lastRotation2d = startRotation;
        this.defaultSpeed = defaultSpeed;
    }


    /**
     * This method will move the robot forward by the specified inches at the specified speed.
     * @param inches How far the robot should move forward (Positive values only)
     * @param speedInchesPerSecond The speed the robot should travel at (Positive values only)
     */
    public void moveForward(@Nonnegative double inches, @Nonnegative double speedInchesPerSecond){

		Path path = new Path(here());

		double pointX = lastTranslation2d.getX() + inches * Math.cos(lastRotation2d.getRadians());
        double pointY = lastTranslation2d.getY() + inches * Math.sin(lastRotation2d.getRadians());
        
        lastTranslation2d = new Translation2D(pointX, pointY);

        path.addPoint(lastTranslation2d, speedInchesPerSecond);
        drive.setAutoPath(path, false);

        while(!drive.isFinished()) if(isDead()) return;
    }

    /**
     * This method will move the robot forward by the specified inches at the default speed.
     * @param inches How far the robot should move forward (Positive values only)
     */
    public void moveForward(@Nonnegative double inches){

		Path path = new Path(here());

		double pointX = lastTranslation2d.getX() + inches * Math.cos(lastRotation2d.getRadians());
        double pointY = lastTranslation2d.getY() + inches * Math.sin(lastRotation2d.getRadians());
        
        lastTranslation2d = new Translation2D(pointX, pointY);

        path.addPoint(lastTranslation2d, defaultSpeed);
        drive.setAutoPath(path, false);

        while(!drive.isFinished()) if(isDead()) return;
    }

    /**
     * This method will move the robot backwards by the specified inches at the specified speed.
     * @param inches How far the robot should move backwards (Positive values only)
     * @param speedInchesPerSecond The speed the robot should travel at (Positive values only)
     */
    public void moveBackward(@Nonnegative double inches, @Nonnegative double speedInchesPerSecond){
        inches = -inches;
        
        Path path = new Path(here());
        

		double pointX = lastTranslation2d.getX() + inches * Math.cos(lastRotation2d.getRadians());
        double pointY = lastTranslation2d.getY() + inches * Math.sin(lastRotation2d.getRadians());
        
        lastTranslation2d = new Translation2D(pointX, pointY);

        path.addPoint(lastTranslation2d, speedInchesPerSecond);
        drive.setAutoPath(path, true);

        while(!drive.isFinished()) if(isDead()) return;
    }

    /**
     * This method will move the robot backwards by the specified inches at the default speed.
     * @param inches How far the robot should move backwards (Positive values only)
     */
    public void moveBackward(@Nonnegative double inches){
        inches = -inches;
        
        Path path = new Path(here());
        

		double pointX = lastTranslation2d.getX() + inches * Math.cos(lastRotation2d.getRadians());
        double pointY = lastTranslation2d.getY() + inches * Math.sin(lastRotation2d.getRadians());
        
        lastTranslation2d = new Translation2D(pointX, pointY);

        path.addPoint(lastTranslation2d, defaultSpeed);
        drive.setAutoPath(path, true);

        while(!drive.isFinished()) if(isDead()) return;
    }

    /**
     * This will turn the robot the specified number of degrees
     * @param degrees How many degrees the robot should turn (Use negative values for left and positive values for right)
     */
    public void turn(double degrees){
        Rotation2D newRotation2d = Rotation2D.fromDegrees(lastRotation2d.getDegrees()+degrees);
        drive.setRotation(newRotation2d);
        lastRotation2d = newRotation2d;
        while(!drive.isFinished()) if(isDead()) return;
	}


    
}
