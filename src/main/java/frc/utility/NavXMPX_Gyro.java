// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavXMPX_Gyro extends AHRS{

	public final int DEFSAMPLES = 5;
	private int samples;
	private double lastHeading;

	public NavXMPX_Gyro(SPI.Port port) {
		super(port);
		super.calibrate();
		samples = DEFSAMPLES;
	}

	public NavXMPX_Gyro(SPI.Port port, int numSamples) {
		super(port);
		samples = numSamples;
	}

	public double calcDiff() {
		return get() - lastHeading;
	}

	public double get() {
		double[] data = new double[samples];
		for (int i = 0; i < samples; i++) {
			data[i] = super.getAngle();
		}

		Arrays.sort(data);
		return data[samples / 2];
	}

	@Override
	public double getAngle() {
		return calcDiff();
	}

	@Override
	public double pidGet() {
		return getAngle();
	}

	public void resetDiff() {
		lastHeading = get();
	}
}
