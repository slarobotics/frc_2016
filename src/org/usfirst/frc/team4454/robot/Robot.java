
package org.usfirst.frc.team4454.robot;


import com.kauailabs.navx.frc.AHRS;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.CameraServer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	Joystick leftStick     = new Joystick(0);
	Joystick rightStick    = new Joystick(1);
	Joystick operatorStick = new Joystick(2);

	Talon frontLeftMotor   = new Talon(0);
	Talon rearLeftMotor    = new Talon(1);
	Talon frontRightMotor  = new Talon(2);
	Talon rearRightMotor   = new Talon(3);

	Talon intake1          = new Talon(4);
	Talon intake2          = new Talon(5);

	RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

	Camera camera = new Camera();

	AHRS ahrs;

	Timer autoTimer = new Timer();
	Timer backupTimer = new Timer();
	boolean backup = false;
	double backupTime = 0.02;
	
	String[] autonomousModes = {"forward", "backward"};
	int autonomousMode = 0;
	boolean autoModeButtonDown = false;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		camera.start();

		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {
		ahrs.zeroYaw();
		autoTimer.reset();
		autoTimer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		double direction = 1;
		switch(autonomousModes[autonomousMode]){
		case "forward":
			direction = 1;
			break;
		case "backward":
			direction = -1;
			break;
		}
		final double driveTime = 4.5; // drive time in seconds
		double power = 0.65;
		SmartDashboard.putBoolean("Moving", ahrs.isMoving());
		if (autoTimer.get() <= driveTime) {
			if(!ahrs.isMoving() && power < 1 && autoTimer.get() > 0){
				backupTimer.reset();
				backupTimer.start();
				backup = true;
				power = 1;
			}
			if(!backup){
				PIDDrive(power * direction);
			}
			reportAHRS();
		} else {
			drive.tankDrive(0, 0);
		}
		while(backup && backupTimer.get() < backupTime){
			PIDDrive(-power * direction);
		}
		if(backupTimer.get() >= backupTime){
			backup = false;
			backupTimer.reset();
		}
	}
	void PIDDrive(double power){
		final double kP = -0.01;
		double yaw = ahrs.getYaw();
		double c = power;       // motor power - common mode
		
		double d = kP * yaw;   // turning amount - differential mode
		double left  = (c+d);
		double right = (c-d);

		drive.tankDrive(left, right);
	}
	public void teleopInit() {		
		camera.enable();
		ahrs.zeroYaw();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double scale = 0.75;

		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 1;
		} else {
			scale = 0.75;
		}
		//drive.tankDrive(-leftStick.getY() * scale, -rightStick.getY() * scale); 
		adaptiveDrive(-leftStick.getY() * scale, -rightStick.getY() * scale);
		
		/*if(operatorStick.getTrigger()){
			this.setIntake(-operatorStick.getY() * 0.1);
		}
		else{
			this.setIntake(-operatorStick.getY());
		}*/
		
		if(Math.abs(operatorStick.getRawAxis(2)) > 0.02)
			this.setIntake(operatorStick.getRawAxis(2));
		else if(Math.abs(operatorStick.getRawAxis(3)) > 0.02)
			this.setIntake(-operatorStick.getRawAxis(3));
		else
			this.setIntake(0);
		reportAHRS();
	}
	
	public void adaptiveDrive(double l, double r){
		// alpha is a parameter between 0 and 1
		final double alpha = 0.5;
		double c = 0.5 * (l+r);
		double d = 0.5 * (l-r);
		double scale = (1 - (alpha * c * c));
		d *= scale;
		l = c + d;
		r = c - d;
		drive.tankDrive(l, r);
	}

	public void setIntake (double level) {
		
		// scale down power for pulling the ball in but allow full power for output.
		if (level > 0) {
			level *= 0.5;
		}
		
		// Make sure to set intake1 and intake2 to spin in opposition
		intake1.set(-level);
		intake2.set(level);
	}

	/**
	 * This function is called when the disabled button is hit.
	 * You can use it to reset subsystems before shutting down.
	 */
	public void disabledInit(){
		if(camera.status)
			camera.disable();
	}
	
	public void disabledPeriodic(){
		if(operatorStick.getRawButton(4) && !autoModeButtonDown){
			autoModeButtonDown = true;
			autonomousMode++;
			if(autonomousMode >= autonomousModes.length){
				autonomousMode = 0;
			}
		}
		if(!operatorStick.getRawButton(4)){
			autoModeButtonDown = false;
		}
		SmartDashboard.putString("Auto Mode", autonomousModes[autonomousMode]);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {

	}

	public void reportAHRS () {

		/* Display 6-axis Processed Angle Data                                      */
		SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
		SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
		SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

		/* Display tilt-corrected, Magnetometer-based heading (requires             */
		/* magnetometer calibration to be useful)                                   */

		SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());

		/* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
		SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

		/* These functions are compatible w/the WPI Gyro Class, providing a simple  */
		/* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */

		SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
		SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());
		
		SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
		SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
		 SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
		SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
		

	}

}
