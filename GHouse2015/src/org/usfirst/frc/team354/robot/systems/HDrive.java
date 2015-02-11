package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SpeedController;

public class HDrive {
	private SpeedController frontLeftSC;
	private SpeedController frontRightSC;
	private SpeedController rearLeftSC;
	private SpeedController rearRightSC;
	private SpeedController hDriveSC;
	
	private RobotDrive robotDrive;
	
	public HDrive(SpeedController fL, SpeedController fR,
				  SpeedController rL, SpeedController rR,
				  SpeedController hD) {
		this.frontLeftSC = fL;
		this.frontRightSC = fR;
		this.rearLeftSC = rL;
		this.rearRightSC = rR;
		this.hDriveSC = hD;
		
		this.robotDrive = new RobotDrive(this.frontLeftSC, this.rearLeftSC, 
										 this.frontRightSC, this.rearRightSC);
		
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
		
	}
	
	//Option for 2 drive motors
	public HDrive(SpeedController fL, SpeedController fR,
			      SpeedController hD) {
		this.frontLeftSC = fL;
		this.frontRightSC = fR;
		this.hDriveSC = hD;
		
		this.robotDrive = new RobotDrive(this.frontLeftSC, this.frontRightSC);
		
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
	}
	
	public void hDrive(double speed, double turn, double strafe) {
		this.robotDrive.arcadeDrive(speed, turn);
		this.hDriveSC.set(strafe);
	}
}
