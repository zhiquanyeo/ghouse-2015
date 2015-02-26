
package org.usfirst.frc.team354.robot;


import org.usfirst.frc.team354.robot.systems.HDrive;
import org.usfirst.frc.team354.robot.systems.LiftSystem;
import org.usfirst.frc.team354.robot.systems.LiftSystem.LiftState;
import org.usfirst.frc.team354.robot.systems.ShelfSystem;
import org.usfirst.frc.team354.robot.systems.ShelfSystem.ShelfState;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/****************************************************
 * G-House Pirates 2015 Robot for Recycle Rush
 * 
 * Drive System:
 * - H-drive
 * TODO: Decide if we are slaving left/right via Y-cable, or
 * if we are setting up 4 separate motors
 * 
 * @author Admin
 *
 ****************************************************/

public class Robot extends SampleRobot {
	/*********************************
	 * Constants
	 *********************************/
	//Motor/Drivetrain ports
	private static final int PORT_MOTOR_FRONT_LEFT = 0; //PWM
	private static final int PORT_MOTOR_FRONT_RIGHT = 1; //PWM
	//private static final int PORT_MOTOR_REAR_LEFT = 2; //PWM - Commenting out since we are slaving left and right sides
	//private static final int PORT_MOTOR_REAR_RIGHT = 3; //PWM
	private static final int PORT_MOTOR_H_DRIVE = 2; //PWM
	
	//Lift System ports
	private static final int PORT_MOTOR_LIFT = 3; //PWM
	private static final int PORT_MOTOR_LIFT_ROLLER_A = 5; //PWM
	private static final int PORT_MOTOR_LIFT_ROLLER_B = 6; //PWM
	private static final int PORT_LIFT_ENCODER_CH_A = 7; //Digital
	private static final int PORT_LIFT_ENCODER_CH_B = 8; //Digital
	private static final int PORT_LIFT_TOP_SWITCH = 0; //Digital
	private static final int PORT_LIFT_BOTTOM_SWITCH = 1; //Digital
	
	//Shelf System ports
	private static final int PORT_MOTOR_SHELF = 4; //PWM
	private static final int PORT_MOTOR_SHELF_ROLLER_A = 7; //PWM
	private static final int PORT_MOTOR_SHELF_ROLLER_B = 8; //PWM
	private static final int PORT_SHELF_ENCODER_CH_A = 5; //Digital
	private static final int PORT_SHELF_ENCODER_CH_B = 6; //Digital
	private static final int PORT_SHELF_OPEN_SWITCH = 2; //Digital
	private static final int PORT_SHELF_CLOSE_SWITCH = 3; //Digital
	
	
	/*********************************
	 * Main Drive System Components
	 *********************************/
	private HDrive robotDrive;
	private RobotDrive robotMecDrive; //Meccanum Drive
	//Speed Controllers for the drive train
	private SpeedController fL;
	private SpeedController fR;
	private SpeedController rL;
	private SpeedController rR;
	private SpeedController hD;
	private double driveExpoValue = 4.0;
	private double crabExpoValue = 4.0; //Separate for strafing
	private double maxTurnSpeed = 0.7; //Scale to this much for the turn speed
	
	/*********************************
	 * Lift Components
	 *********************************/
	private LiftSystem liftSystem;
	private SpeedController liftMotor;
	private SpeedController liftRollerA;
	private SpeedController liftRollerB;
	private Encoder liftEncoder;
	private DigitalInput liftTopSwitch;
	private DigitalInput liftBottomSwitch;
	
	/*********************************
	 * Shelf Components
	 *********************************/
	private ShelfSystem shelfSystem;
	private SpeedController shelfMotor;
	private SpeedController shelfRollerA;
	private SpeedController shelfRollerB;
	private Encoder shelfEncoder;
	private DigitalInput shelfOpenSwitch;
	private DigitalInput shelfCloseSwitch;
	
	/*********************************
	 * HID Control Components
	 *********************************/
	Joystick driveController = new Joystick(0);
	Joystick codriverController = new Joystick(1);
	
    public Robot() {
    	//Initialize all our subsystems
    	initializeDriveSystem();
    	initializeLiftSystem();
    	initializeShelfSystem();
    }

    /**
     * Initialize the drive system
     */
    private void initializeDriveSystem() {
    	//If we need to switch SpeedController types, do it here
    	//By default, we will use Victor 888s
    	//We can also end up using less speed controller instances
    	//by slaving both left side motors together
    	fL = new Victor(PORT_MOTOR_FRONT_LEFT);
    	fR = new Victor(PORT_MOTOR_FRONT_RIGHT);
    	//rL = new Victor(PORT_MOTOR_REAR_LEFT);
    	//rR = new Victor(PORT_MOTOR_REAR_RIGHT);
    	hD = new Victor(PORT_MOTOR_H_DRIVE);
    	
    	robotDrive = new HDrive(fL, fR, hD);
    	
    	//robotMecDrive = new RobotDrive(fL, rL, fR, rR);
    }
    
    private void initializeLiftSystem() {
    	liftMotor = new Victor(PORT_MOTOR_LIFT);
    	liftEncoder = new Encoder(PORT_LIFT_ENCODER_CH_A, PORT_LIFT_ENCODER_CH_B, false, EncodingType.k4X);
    	liftTopSwitch = new DigitalInput(PORT_LIFT_TOP_SWITCH);
    	liftBottomSwitch = new DigitalInput(PORT_LIFT_BOTTOM_SWITCH);
    	
    	liftRollerA = new Talon(PORT_MOTOR_LIFT_ROLLER_A);
    	liftRollerB = new Talon(PORT_MOTOR_LIFT_ROLLER_B);
    	
    	liftSystem = new LiftSystem(liftMotor, liftTopSwitch, liftBottomSwitch, liftEncoder, liftRollerA, liftRollerB);
    }
    
    private void initializeShelfSystem() {
    	shelfMotor = new Victor(PORT_MOTOR_SHELF);
    	shelfEncoder = new Encoder(PORT_SHELF_ENCODER_CH_A, PORT_SHELF_ENCODER_CH_B);
    	shelfOpenSwitch = new DigitalInput(PORT_SHELF_OPEN_SWITCH);
    	shelfCloseSwitch = new DigitalInput(PORT_SHELF_CLOSE_SWITCH);
    	shelfRollerA = new Talon(PORT_MOTOR_SHELF_ROLLER_A);
    	shelfRollerB = new Talon(PORT_MOTOR_SHELF_ROLLER_B);
    	
    	shelfSystem = new ShelfSystem(shelfMotor, shelfOpenSwitch, shelfCloseSwitch, shelfEncoder, shelfRollerA, shelfRollerB);
    }
    
    public void autonomous() {
        
    }
    
    //Move this up at some point
    enum AutoGateModeState {
    	IDLE,
    	LIFT_RAISING,
    	LIFT_RAISED,
    	SHELF_OPENING,
    	SHELF_OPEN
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
    	//liftSystem.sendToBottom();
    	
    	boolean autoGateMode = false;
    	AutoGateModeState gateModeState = AutoGateModeState.IDLE;
    	
        while (isOperatorControl() && isEnabled()) {
        	// Inform the various systems that we're in a new cycle
        	liftSystem.update();
        	shelfSystem.update();
        	
        	//DRIVE THIS THING
        	doDrive();
        	
        	if (driveController.getRawButton(8) && !autoGateMode) {
        		autoGateMode = true;
        		gateModeState = AutoGateModeState.LIFT_RAISING;
        		liftSystem.moveToSafeHeight();
        	}
        	
        	if (autoGateMode) {
        		if (gateModeState == AutoGateModeState.LIFT_RAISING && liftSystem.getState() == LiftState.AT_POINT) {
        			gateModeState = AutoGateModeState.SHELF_OPENING;
        			shelfSystem.fullyOpen();
        		}
        		if (gateModeState == AutoGateModeState.SHELF_OPENING && shelfSystem.getState() == ShelfState.OPEN) {
        			gateModeState = AutoGateModeState.SHELF_OPEN;
        			autoGateMode = false;
        		}
        	}
        	
        	//==== Lift Controls (Manual Override) ====
        	if (driveController.getRawButton(2) || driveController.getRawButton(4)) {
        		if (driveController.getRawButton(2)) {
        			liftSystem.moveDown();
        		}
        		else {
        			liftSystem.moveUp();
        		}
        	}
        	else if (liftSystem.getState() != LiftState.MOVING_TO_BOTTOM &&
        			 liftSystem.getState() != LiftState.MOVING_TO_TOP &&
        			 liftSystem.getState() != LiftState.MOVING_TO_SAFE_POINT) {
        		liftSystem.stop();
        	}
        	
        	//==== Shelf Controls (Manual Override) ====
        	if (driveController.getRawButton(1) || driveController.getRawButton(3)) {
        		if (driveController.getRawButton(1)) {
        			shelfSystem.open();
        		}
        		else {
        			shelfSystem.close();
        		}
        	}
        	else if (shelfSystem.getState() != ShelfState.MOVING_TO_CLOSED &&
        			 shelfSystem.getState() != ShelfState.MOVING_TO_OPEN) {
        		shelfSystem.stop();
        	}
        	
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
    
    //===== HELPER FUNCTIONS
    //Exponential scaling function
    public double expo(double input, double expoValue) {
        double multiplier = 1.0;
        if (input < 0) {
            input = input * -1.0;
            multiplier = -1.0;
        }
        double yVal = (Math.exp(expoValue * input) - 1) / (Math.exp(expoValue) - 1);
        return multiplier * yVal;
    }
    
    /**
     * Helper method for driving. Takes into account exponent values and
     * potentially control mode
     */
    private void doDrive() {
    	double crabVal, driveVal, turnVal;
        crabVal = driveController.getX();
        driveVal = driveController.getY();
        turnVal = driveController.getZ();
        
        crabVal = expo(crabVal, crabExpoValue);
        driveVal = expo(driveVal, driveExpoValue);
        turnVal = expo(turnVal, driveExpoValue);
        
        //Clamp the turn speed
        turnVal *= maxTurnSpeed;
        
    	robotDrive.hDrive(-driveVal, -turnVal, -crabVal);
    }
}
