package org.usfirst.frc.team4454.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.hal.AnalogJNI;

public class MaxBotix extends AnalogInput {

	public MaxBotix(int channel) {
		super(channel);
		// TODO Auto-generated constructor stub
	}

	public double getVoltage() {
	    return super.getAverageVoltage() / 512;
	  }
	
}
