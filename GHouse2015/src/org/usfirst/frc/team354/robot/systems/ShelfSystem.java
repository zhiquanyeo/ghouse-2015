package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;

public class ShelfSystem {
	private SpeedController shelfMotor;
	private DigitalInput openSwitch;
	private DigitalInput closeSwitch;
	
	public ShelfSystem(SpeedController sMotor, DigitalInput oSwitch, DigitalInput cSwitch) {
		shelfMotor = sMotor;
		openSwitch = oSwitch;
		closeSwitch = cSwitch;
	}
}
