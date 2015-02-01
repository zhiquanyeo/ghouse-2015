
package org.usfirst.frc.team354.robot;


import org.usfirst.frc.team354.robot.systems.HDrive;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
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
	private static final int PORT_MOTOR_FRONT_LEFT = 1;
	private static final int PORT_MOTOR_FRONT_RIGHT = 2;
	private static final int PORT_MOTOR_REAR_LEFT = 3;
	private static final int PORT_MOTOR_REAR_RIGHT = 4;
	private static final int PORT_MOTOR_H_DRIVE = 5;
	
	//Lift System ports
	private static final int PORT_MOTOR_LIFT = 6;
	
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
	
	/*********************************
	 * Lift Components
	 *********************************/
	private SpeedController liftMotor;
	
	/*********************************
	 * HID Control Components
	 *********************************/
	Joystick driveController;
	
	//TODO: Get rid
	RobotDrive driveTrain;
    RobotDrive myRobot;
    Joystick stick;

    public Robot() {
    	//Initialize all our subsystems
    	initializeDriveSystem();
    	
    	
    	//TODO: Get rid
        myRobot = new RobotDrive(0, 1);
        myRobot.setExpiration(0.1);
        stick = new Joystick(0);
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
    }
    
    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
        myRobot.setSafetyEnabled(false);
        myRobot.drive(-0.5, 0.0);	// drive forwards half speed
        Timer.delay(2.0);		//    for 2 seconds
        myRobot.drive(0.0, 0.0);	// stop robot
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
            myRobot.arcadeDrive(stick); // drive with arcade style (use right stick)
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}
