package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * The ShelfSystem class represents the physical swinging shelf mechanism on the robot.
 * The shelf mechanism consists of a belt driven cylinder (driven by shelfMotor),
 * connected directly to the shelf. There are limit switches (openSwitch and closeSwitch)
 * on the mechanism that allow the robot to know when the shelf is at the physical stop
 * limits. The shelf is fully OPEN when it's plate is horizontal, and fully CLOSE when it 
 * is resting against the backstop.
 * 
 * The shelf drive motor (should) also include a quadrature encoder. The use for this is
 * TBD at the moment.
 * 
 * There are also 2 roller motors (rollerA and rollerB) that act as intake feeds.
 * 
 * The ShelfSystem operates as a state machine. The basic states are:
 * - CLOSED : The shelf is in the fully closed position.
 * - AT_POINT : The shelf is stopped at some point between OPEN and CLOSED
 * - OPEN : The shelf is in the fully open position
 * - MOVING_TO_POINT : The shelf is currently moving to a point (arbitrary)
 * - MOVING_TO_OPEN : The shelf system is moving to the open position
 * - MOVING_TO_CLOSED : The shelf system is moving to the closed position
 * 
 * CLOSED, AT_POINT and OPEN are valid start and end states. The ShelfSystem should start
 * at the AT_POINT state until it can positively ascertain that it is either CLOSED or OPEN.
 * 
 * Special point is at -272
 */
public class ShelfSystem {
	private SpeedController shelfMotor;
	private DigitalInput openSwitch;
	private DigitalInput closeSwitch;
	private Encoder shelfEncoder;
	private SpeedController rollerA;
	private SpeedController rollerB;
	
	private static final double MOTOR_SPEED = 0.5; //This controls how fast the shelf opens and closes
	private static final double ROLLER_SPEED = 0.6; //This controls how fast the rollers spin
	
	//===== State Machine =====
	
	public enum ShelfState {
		CLOSED,
		OPEN,
		AT_POINT,
		MOVING_TO_POINT,
		MOVING_TO_OPEN,
		MOVING_TO_CLOSED,
		MOVING_TO_SWEET_SPOT
	};
	
	private ShelfState currentState = ShelfState.AT_POINT;
	
	private long lastUpdateTime = 0;
	
	public ShelfSystem(SpeedController sMotor, DigitalInput oSwitch, DigitalInput cSwitch, Encoder sEncoder, SpeedController rA, SpeedController rB) {
		shelfMotor = sMotor;
		openSwitch = oSwitch;
		closeSwitch = cSwitch;
		shelfEncoder = sEncoder;
		rollerA = rA;
		rollerB = rB;
	}
	
	public void update() {
		long currentTime = System.currentTimeMillis();
		
		/**
		 * Update the state machine here
		 * If we were forced to move to open/closed via fullyOpen/fullyClose
		 * then we keep the motor running until we hit one of the switches
		 * and then transition state.
		 * 
		 * If we are moving to a point, check if we have hit one of the switches
		 * and transition state accordingly.
		 */
		 if (currentState == ShelfState.MOVING_TO_OPEN) {
		 	//Stop the motor if we have hit the open switch
		 	if (!openSwitch.get()) {
		 		shelfMotor.set(0);
		 		currentState = ShelfState.OPEN;
		 	}
		 }
		 else if (currentState == ShelfState.MOVING_TO_CLOSED) {
		 	//Stop the motor if we have hit the closed switch
		 	if (!closeSwitch.get()) {
		 		shelfMotor.set(0);
		 		currentState = ShelfState.CLOSED;
		 	}
		 }
		 else if (currentState == ShelfState.MOVING_TO_POINT) {
//		 	if (!openSwitch.get()) {
//		 		shelfMotor.set(0);
//		 		currentState = ShelfState.OPEN;
//		 	}
//		 	else if (!closeSwitch.get()) {
//		 		shelfMotor.set(0);
//		 		currentState = ShelfState.CLOSED;
//		 	}
		 }
		 else if (currentState == ShelfState.MOVING_TO_SWEET_SPOT) {
			 if (shelfEncoder.get() >= -270 && shelfEncoder.get() <= -265) {
				 shelfMotor.set(0);
				 currentState = ShelfState.AT_POINT;
			 }
			 else {
				 if (shelfEncoder.get() < -270) {
						//move in the close direction
					 	System.out.println("Moving in close direction (update)");
						shelfMotor.set(MOTOR_SPEED);
						//currentState = ShelfState.MOVING_TO_SWEET_SPOT;
					}
					else if (shelfEncoder.get() > -265) {
						System.out.println("Moving in open direction (update)");
						shelfMotor.set(-MOTOR_SPEED);
						//currentState = ShelfState.MOVING_TO_SWEET_SPOT;
					}
			 }
		 }
		
		lastUpdateTime = currentTime;
	}
	
	public void fullyOpen() {
		//Only run this if we are at a point
		if (currentState == ShelfState.AT_POINT || currentState == ShelfState.CLOSED) {
			//Run the motor until the open switch registers off
			shelfMotor.set(-MOTOR_SPEED); //TODO: If the motor runs in the opposite direction, flip this
			currentState = ShelfState.MOVING_TO_OPEN;
		}
	}
	
	public void fullyClose() {
		//Only run this if we are at a point
		if (currentState == ShelfState.AT_POINT || currentState == ShelfState.OPEN) {
			//Run the motor until the open switch registers off
			shelfMotor.set(MOTOR_SPEED); //TODO: If the motor runs in the opposite direction, flip this
			currentState = ShelfState.MOVING_TO_CLOSED;
		}
	}
	
	public void open() {
		if (!openSwitch.get()) {
			System.out.println("NOT OK");
			shelfMotor.set(0);
			currentState = ShelfState.OPEN;
		}
		else {
			System.out.println("OK");
			shelfMotor.set(-MOTOR_SPEED); //TODO If we're spinnign the wrong way, flip this
			currentState = ShelfState.MOVING_TO_POINT;
			System.out.println("ShelfEncoder: " + shelfEncoder.get());
		}
		
	}
	
	public void close() {
		if (!closeSwitch.get()) {
			shelfMotor.set(0);
			currentState = ShelfState.CLOSED;
			shelfEncoder.reset();
			System.out.println("shelf encoder reset");
		}
		else {
			shelfMotor.set(MOTOR_SPEED); //TODO If we're spinnign the wrong way, flip this
			currentState = ShelfState.MOVING_TO_POINT;
			System.out.println("ShelfEncoder: " + shelfEncoder.get());
		}
		
	}
	
	public void stop() {
		shelfMotor.set(0);
		if (!openSwitch.get()) {
			currentState = ShelfState.OPEN;
		}
		else if (!closeSwitch.get()) {
			currentState = ShelfState.CLOSED;
		}
		else {
			currentState = ShelfState.AT_POINT;
		}
	}
	
	public void moveToSweetSpot() {
		//-272. more negative = more open
		System.out.println("Moving to sweet spot. curr encoder: " + shelfEncoder.get());
		if (shelfEncoder.get() < -270) {
			System.out.println("Moving in Close direction");
			//move in the close direction
			shelfMotor.set(MOTOR_SPEED);
			currentState = ShelfState.MOVING_TO_SWEET_SPOT;
		}
		else if (shelfEncoder.get() > -265) {
			System.out.println("Moving in open direction");
			shelfMotor.set(-MOTOR_SPEED);
			currentState = ShelfState.MOVING_TO_SWEET_SPOT;
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
	
	public ShelfState getState() {
		return currentState;
	}
}
