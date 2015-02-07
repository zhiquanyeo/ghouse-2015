
package org.usfirst.frc.team354.robot;


import org.usfirst.frc.team354.robot.systems.HDrive;

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
	private static final int PORT_MOTOR_REAR_LEFT = 2; //PWM
	private static final int PORT_MOTOR_REAR_RIGHT = 3; //PWM
	private static final int PORT_MOTOR_H_DRIVE = 4; //PWM
	
	//Lift System ports
	private static final int PORT_MOTOR_LIFT = 6; //PWM
	private static final int PORT_LIFT_ENCODER_CH_A = 0; //Digital
	private static final int PORT_LIFT_ENCODER_CH_B = 1; //Digital
	private static final int PORT_LIFT_TOP_SWITCH = 2; //Digital
	private static final int PORT_LIFT_BOTTOM_SWITCH = 3; //Digital
	
	/*********************************
	 * Main Drive System Components
	 *********************************/
	private HDrive robotDrive;
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
	private SpeedController liftMotor;
	private Encoder liftEncoder;
	private DigitalInput liftTopSwitch;
	private DigitalInput liftBottomSwitch;
	
	/*********************************
	 * HID Control Components
	 *********************************/
	Joystick driveController = new Joystick(0);
	
	//EXPT
	Servo panServo = new Servo(8);
	Servo tiltServo = new Servo(9);
	
    public Robot() {
    	//Initialize all our subsystems
    	initializeDriveSystem();
    	initializeLiftSystem();
    	
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
    	rL = new Victor(PORT_MOTOR_REAR_LEFT);
    	rR = new Victor(PORT_MOTOR_REAR_RIGHT);
    	hD = new Victor(PORT_MOTOR_H_DRIVE);
    	
    	robotDrive = new HDrive(fL, fR, rL, rR, hD);
    }
    
    private void initializeLiftSystem() {
    	liftMotor = new Talon(PORT_MOTOR_LIFT);
    	liftEncoder = new Encoder(PORT_LIFT_ENCODER_CH_A, PORT_LIFT_ENCODER_CH_B);
    	liftTopSwitch = new DigitalInput(PORT_LIFT_TOP_SWITCH);
    	liftBottomSwitch = new DigitalInput(PORT_LIFT_BOTTOM_SWITCH);
    }
    
    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
        while(isAutonomous() && isEnabled()) {
        	panServo.set(0.5);
        	tiltServo.set(0.5);
        	for (int angle = 0; angle < 180; angle += 5) {
        		panServo.setAngle(angle);
        		tiltServo.setAngle(angle);
        		Timer.delay(0.2);
        	}
        }
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	//DRIVE THIS THING
        	doDrive();
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
