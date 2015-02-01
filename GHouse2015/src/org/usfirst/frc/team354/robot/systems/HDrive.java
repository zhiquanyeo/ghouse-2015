package org.usfirst.frc.team354.robot.systems;

import edu.wpi.first.wpilibj.RobotDrive;
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
		
	}
	
	//Option for 2 drive motors
	public HDrive(SpeedController fL, SpeedController fR,
			      SpeedController hD) {
		this.frontLeftSC = fL;
		this.frontRightSC = fR;
		this.hDriveSC = hD;
		
		this.robotDrive = new RobotDrive(this.frontLeftSC, this.frontRightSC);
	}
	
	public void hDrive(double speed, double turn, double strafe) {
		this.robotDrive.arcadeDrive(speed, turn);
		this.hDriveSC.set(strafe);
	}
}
