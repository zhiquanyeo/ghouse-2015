package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * The LiftSystem class represents the physical lift mechanism on the robot.
 * The lift mechanism consists of a belt driven cylinder (driven by liftMotor), 
 * connected via chains to both sides of the lift carriage. There are limit
 * switches (topSwitch and bottomSwitch) on the lift mechanism that allow the robot
 * to know when the carriage is at the physical stop limits.
 * 
 * The lift drive motor also includes a quadrature encoder. This will be used to determine
 * where the carriage is on the lift. This gets reset to 0 every time we bring the carriage
 * all the way to the bottom of the lift.
 * 
 * There are also 2 roller motors (rollerA and rollerB) that act as intake feeds.
 * 
 * The LiftSystem operates as a state machine. The basic states are:
 * - AT_BOTTOM : The lift carriage is at the bottom and contacting the limit switch
 * - AT_POINT : The lift carriage is at some point on the rails (neither top nor bottom)
 * - AT_TOP : The lift carriage is at the top and contacting the limit switch
 * - MOVING_TO_POINT : The lift carriage is currently moving to a point (top, bottom, arbitrary)
 * 
 * The 3 AT_* states are all valid start and end states. The LiftSystem should start at the AT_POINT
 * state until it can positively ascertain that it is either AT_TOP or AT_BOTTOM.
 * 
 * The Safe limit is 6000
 * @author zhiquan
 *
 */
public class LiftSystem {
	private DigitalInput topSwitch;
	private DigitalInput bottomSwitch;
	private SpeedController liftMotor;
	private Encoder liftEncoder;
	private SpeedController rollerA;
	private SpeedController rollerB;
	
	private static final double MOTOR_SPEED = 1.0;
	private static final double ROLLER_SPEED = 0.6;
	
	//====== State Machine =======
	
	public enum LiftState {
		AT_BOTTOM,
		AT_TOP,
		AT_POINT,
		MOVING_TO_POINT,
		MOVING_TO_TOP,
		MOVING_TO_BOTTOM,
		MOVING_TO_SAFE_POINT
	};
	
	private LiftState currentState = LiftState.AT_POINT;
	
	private long lastUpdateTime = 0;
	
	public LiftSystem(SpeedController lMotor, DigitalInput tSwitch, DigitalInput bSwitch, Encoder lEncoder, SpeedController rA, SpeedController rB) {
		liftMotor = lMotor;
		topSwitch = tSwitch;
		bottomSwitch = bSwitch;
		liftEncoder = lEncoder;
		rollerA = rA;
		rollerB = rB;
	}
	
	public void update() {
		long currentTime = System.currentTimeMillis();
		/**
		 * We update the state machine here
		 * If we were forced to move to top/bottom via sendToTop/sendToBottom
		 * Then we keep the motor running until we hit one of the switches
		 * and then transition state.
		 * 
		 * If we are moving to a point, check if we have hit one of the switches
		 * and transition state accordingly.
		 */
		if (currentState == LiftState.MOVING_TO_TOP) {
			//Stop the motor if we have hit the top switch
			if (!topSwitch.get()) {
				liftMotor.set(0);
				currentState = LiftState.AT_TOP;
			}
			else {
				liftMotor.set(MOTOR_SPEED);
			}
		}
		else if (currentState == LiftState.MOVING_TO_BOTTOM) {
			//Stop the motor if we have hit the bottom switch
			if (!bottomSwitch.get()) {
				liftMotor.set(0);
				currentState = LiftState.AT_BOTTOM;
				liftEncoder.reset();
			}
			else {
				System.out.println("Moving to bottom...");
				liftMotor.set(-MOTOR_SPEED);
				System.out.println("MotorSpeed set");
				currentState = LiftState.MOVING_TO_BOTTOM;
			}
		}
		else if (currentState == LiftState.MOVING_TO_POINT) {
//			if (!topSwitch.get()) {
//				liftMotor.set(0);
//				currentState = LiftState.AT_TOP;
//			}
//			else if (!bottomSwitch.get()) {
//				liftMotor.set(0);
//				currentState = LiftState.AT_BOTTOM;
//			}
		}
		else if (currentState == LiftState.MOVING_TO_SAFE_POINT) {
			if (liftEncoder.get() > 6000) {
				liftMotor.set(0);
				currentState = LiftState.AT_POINT;
			}
			else {
				liftMotor.set(MOTOR_SPEED);
			}
		}
		
		lastUpdateTime = currentTime;
	}
	
	public void sendToBottom() {
		//Only run this if we are at a point
		//if (currentState == LiftState.AT_POINT || currentState == LiftState.AT_TOP) { 
			//Run the motor until the bottom switch registers off
			liftMotor.set(-MOTOR_SPEED);
			currentState = LiftState.MOVING_TO_BOTTOM;
		//}
	}
	
	public void sendToTop() {
		//if (currentState == LiftState.AT_POINT || currentState == LiftState.AT_BOTTOM) { 
			liftMotor.set(MOTOR_SPEED);
			currentState = LiftState.MOVING_TO_TOP;
		//}
	}
	
	public void calibrate() {
		
	}
	
	/**
	 * Move the lift up. This can override a sendToTop/sendToBottom command
	 */
	public void moveUp() {
		if (!topSwitch.get()) {
			liftMotor.set(0);
			currentState = LiftState.AT_TOP;
		}
		else {
			liftMotor.set(MOTOR_SPEED);
			currentState = LiftState.MOVING_TO_POINT;
			//System.out.println("Encoder: " + liftEncoder.get());
		}
	}
	
	/**
	 * Move the lift down. This can override a sendToTop/sendToBottom command
	 */
	public void moveDown() {
		if (!bottomSwitch.get()) {
			liftMotor.set(0);
			currentState = LiftState.AT_BOTTOM;
			liftEncoder.reset();
		}
		else {
			liftMotor.set(-MOTOR_SPEED);
			currentState = LiftState.MOVING_TO_POINT;
			//System.out.println("Encoder: " + liftEncoder.get());
		}
	}
	
	public void stop() {
		liftMotor.set(0);
		
		if (!bottomSwitch.get()) {
			currentState = LiftState.AT_BOTTOM;
		}
		else if (!topSwitch.get()) {
			currentState = LiftState.AT_TOP;
		}
		else {
			System.out.println("Stopping AT POINT");
			currentState = LiftState.AT_POINT;
		}
	}
	
	public void moveToSafeHeight() {
		//6000
		if (liftEncoder.get() < 6000) {
			//move up
			liftMotor.set(MOTOR_SPEED);
			currentState = LiftState.MOVING_TO_SAFE_POINT;
		}
	}
	
	public void startRollers() {
		rollerA.set(ROLLER_SPEED);
		rollerB.set(-ROLLER_SPEED);
	}
	
	public void stopRollers() {
		rollerA.set(0);
		rollerB.set(0);
	}
	
	public LiftState getState() {
		return currentState;
	}
}
