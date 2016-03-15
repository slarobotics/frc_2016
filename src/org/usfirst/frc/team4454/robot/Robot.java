
package org.usfirst.frc.team4454.robot;


import org.usfirst.frc.team4454.robot.Robot.AutoRoutineEnum;
import org.usfirst.frc.team4454.robot.Robot.AutonModeEnum;

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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;


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

	Camera camera;

	AHRS ahrs;

	Timer autoTimer = new Timer();
	Timer backupTimer = new Timer();
	boolean backup = false;
	double backupTime = 0.02;
	
	DigitalInput BallSensor = new DigitalInput(0);

	// In order to convert the analog voltage to distance in inches divide by 0.0098 or multiply by 5 and divide by 512.
	AnalogInput MaxBotixX = new AnalogInput(0);
	AnalogInput MaxBotixY = new AnalogInput(1);
	
	// Different autonomous routines
	public enum AutoRoutineEnum {BREACH, AUTO_SCORE};
	
	// Different autonomous modes
	public enum AutonModeEnum {GO_STRAIGHT, GO_BACK, APPROACH_WALL, TURN, APPROACH_GOAL, SCORE_BALL};
	
	AutoRoutineEnum AutoRoutine = AutoRoutineEnum.BREACH;
	AutonModeEnum AutonMode = AutonModeEnum.GO_STRAIGHT;

	String[] autonomousModes = {"forward", "backward"};
	int autonomousMode = 0;
	
	double autoPower = 0.65;
	final double driveTime = 5.5;
	
	boolean autoModeButtonDown = false;
	
	double turnAngle = 0;
	double fieldLengthX = 319.72;
	double fieldLengthY = 191.69;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		System.out.println("Starting camera thread");
		camera = new Camera();
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
	
	double getMaxBotixValue(AnalogInput maxBotix){
		return maxBotix.getAverageVoltage() / 0.0098;
	}
	
	boolean checkGoalLineReached(double tolerance){
		return getMaxBotixValue(MaxBotixY) <= (-131.18 / 163.13 * getMaxBotixValue(MaxBotixX) + 113.32) + tolerance && getMaxBotixValue(MaxBotixY) >= (-131.18 / 163.13 * getMaxBotixValue(MaxBotixX) + 113.32) - tolerance;
	}
	
	@Override
	public void autonomousPeriodic() {
		reportAHRS();
		reportSensors();
		switch (AutoRoutine) {
		case BREACH : auto_BREACH(); break;
		case AUTO_SCORE : auto_AUTO_SCORE(); break;
		}	
	}
	
	void auto_STRAIGHT () {
		PIDDrive(autoPower);
		if (!ahrs.isMoving() && autoPower < 1 && autoTimer.get() > 0) {
			AutonMode = AutonModeEnum.GO_BACK;
			backupTimer.reset();
			backupTimer.start();
			autoPower = 1;
		}
	}
	
	
	void auto_BREACH () {
		switch (AutonMode) {
		
		case GO_STRAIGHT : 
			System.out.println("Going Straight in Auto Breach");
			if (autoTimer.get() < driveTime) {
				auto_STRAIGHT();
			}
			else{
				PIDDrive(0);
			}
			break;
		case GO_BACK : 
			System.out.println("Going Back in Auto Breach");
			if(backupTimer.get() < backupTime){
				PIDDrive(-autoPower);
			}
			else {
				AutonMode = AutonModeEnum.GO_STRAIGHT;
			}
		}
		
	}

	void auto_AUTO_SCORE () {
		switch(AutonMode){
		case GO_STRAIGHT:
			System.out.println("Going Straight in Auto Score");
			if(autoTimer.get() < driveTime){
				auto_STRAIGHT();
				if(checkGoalLineReached(2)){
					AutonMode = AutonModeEnum.TURN;
					turnAngle = Math.atan((fieldLengthX / 2 - getMaxBotixValue(MaxBotixX)) / getMaxBotixValue(MaxBotixY));
					ahrs.zeroYaw();
					AutonMode = AutonModeEnum.TURN;
				}
			}		
		break;
		case GO_BACK:
			System.out.println("Going Back in Auto Score");
			if(backupTimer.get() < backupTime){
				PIDDrive(-autoPower);
			}
			else {
				AutonMode = AutonModeEnum.GO_STRAIGHT;
			}
		break;
		case TURN:
			System.out.println("Turning in Auto Score");
			while(ahrs.getYaw() < turnAngle){
				drive.tankDrive(0.5, 0);
			}
			AutonMode = AutonModeEnum.APPROACH_GOAL;
			break;
		case APPROACH_GOAL:
			System.out.println("Approaching goal in Auto Score");
			while(getMaxBotixValue(MaxBotixY) > 7){
				PIDDrive(autoPower);
			}
			AutonMode = AutonModeEnum.SCORE_BALL;
			break;
		case SCORE_BALL:
			System.out.println("Scoring in Auto Score");
			setIntake(-1);
			break;
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
		reportAHRS();
		reportSensors();
		
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
		if(level > 0 && !BallSensor.get() && !operatorStick.getRawButton(3))
			return;
		
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
		camera.disable();
	}
	
	public void disabledPeriodic(){
		int idx;
		
		if(operatorStick.getRawButton(4) && !autoModeButtonDown){
			autoModeButtonDown = true;
			
			// Get current index
			idx = AutoRoutine.ordinal();
			
			// Get index of next choice with wraparound
			idx = (idx + 1) % (AutoRoutineEnum.values().length);
			
			AutoRoutine = AutoRoutineEnum.values()[idx];
		}
		if(!operatorStick.getRawButton(4)){
			autoModeButtonDown = false;
		}
		
		SmartDashboard.putString("Auto Routine", AutoRoutine.name());
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {

	}
	
	public void reportSensors() {
		SmartDashboard.putBoolean(  "Ball Sensor",   BallSensor.get());
		SmartDashboard.putNumber(   "MaxBotix X",   getMaxBotixValue(MaxBotixX));
		SmartDashboard.putNumber(   "MaxBotix Y",   getMaxBotixValue(MaxBotixY));
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
