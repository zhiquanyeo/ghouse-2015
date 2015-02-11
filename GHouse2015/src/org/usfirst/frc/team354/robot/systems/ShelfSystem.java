package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class ShelfSystem {
	private SpeedController shelfMotor;
	private DigitalInput openSwitch;
	private DigitalInput closeSwitch;
	private Encoder shelfEncoder;
	private SpeedController rollerA;
	private SpeedController rollerB;
	
	public ShelfSystem(SpeedController sMotor, DigitalInput oSwitch, DigitalInput cSwitch, Encoder sEncoder, SpeedController rA, SpeedController rB) {
		shelfMotor = sMotor;
		openSwitch = oSwitch;
		closeSwitch = cSwitch;
		shelfEncoder = sEncoder;
		rollerA = rA;
		rollerB = rB;
	}
	
	public void update() {
		
	}
}
