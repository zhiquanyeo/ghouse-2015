package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class LiftSystem {
	private DigitalInput topSwitch;
	private DigitalInput bottomSwitch;
	private SpeedController liftMotor;
	private Encoder liftEncoder;
	
	private static final double MOTOR_SPEED = 0.4;
	
	public LiftSystem(SpeedController lMotor, DigitalInput tSwitch, DigitalInput bSwitch, Encoder lEncoder) {
		liftMotor = lMotor;
		topSwitch = tSwitch;
		bottomSwitch = bSwitch;
		lEncoder = liftEncoder;
	}
	
	public void update() {
		
	}
	
	public void sendToBottom() {
		//Run the motor until the bottom switch registers off
		while (bottomSwitch.get()) {
			liftMotor.set(MOTOR_SPEED);
		}
		
		//Reset the encoder
		liftEncoder.reset();
	}
	
	public void sendToTop() {
		while (topSwitch.get()) {
			liftMotor.set(-MOTOR_SPEED);
		}
	}
	
	public void calibrate() {
		sendToBottom();
		sendToTop();
		sendToBottom();
		
		int encoderCount = liftEncoder.get();
		//Do some math here
	}
}
