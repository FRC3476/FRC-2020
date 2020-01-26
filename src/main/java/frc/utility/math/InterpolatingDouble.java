// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.math;

/**
 * Class that implements Interpolable for use in InterpolableValue
 */
public class InterpolatingDouble implements Interpolable<InterpolatingDouble> {

	private double value;

	public InterpolatingDouble(double value) {
		this.value = value;
	}

	public double getValue() {
		return value;
	}

	@Override
	public InterpolatingDouble interpolate(InterpolatingDouble other, double percentage) {
		double diff = other.getValue() - value;
		return new InterpolatingDouble(value + (diff * percentage));
	}
}
