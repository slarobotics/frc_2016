
package org.usfirst.frc.team4454.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
//import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;


import edu.wpi.first.wpilibj.IterativeRobot;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// import com.ni.vision.NIVision;
// import com.ni.vision.NIVision.Image;
// import edu.wpi.first.wpilibj.CameraServer;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.CANTalon;
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
	
	//CANTalon intakeHeight = new CANTalon(1);
	//CANTalon intakeHeight2 = new CANTalon(2);
	
	PowerDistributionPanel PDP = new PowerDistributionPanel(0);

	RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

	//Camera camera;

	AHRS ahrs;

	Timer autoTimer = new Timer();
	Timer backupTimer = new Timer();
	boolean backup = false;
	double backupTime = 2;
	
	DigitalInput BallSensor = new DigitalInput(0);

	// In order to convert the analog voltage to distance in inches divide by 0.0098 or multiply by 5 and divide by 512.
	AnalogInput MaxBotixX = new AnalogInput(0);
	AnalogInput MaxBotixY = new AnalogInput(1);
	
	// Different autonomous routines
	public enum AutoRoutineEnum {BREACH, SIMPLE};
	
	// Different autonomous modes
	public enum AutonModeEnum {GO_STRAIGHT, GO_BACK, APPROACH_WALL, TURN, APPROACH_GOAL, SCORE_BALL, END};
	
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

		System.out.println("Enabling camera");
		CameraServer.getInstance().setQuality(50);
		CameraServer.getInstance().startAutomaticCapture("cam0");
		//camera = new Camera();
		//camera.start();

		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		
		/*intakeHeight2.changeControlMode(TalonControlMode.Follower);
		intakeHeight2.set(intakeHeight.getDeviceID());
		int absolutePosition = intakeHeight.getPulseWidthPosition() & 0xFFF;
		intakeHeight.setEncPosition(absolutePosition);
		intakeHeight.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		intakeHeight.reverseSensor(false);
		//intakeHeight.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
		intakeHeight.changeControlMode(TalonControlMode.Position);
		intakeHeight.configNominalOutputVoltage(+0f, -0f);
		intakeHeight.configPeakOutputVoltage(+12f, -12f);
		intakeHeight.setAllowableClosedLoopErr(100);
		intakeHeight.setProfile(0);
		intakeHeight.setF(0);
		intakeHeight.setPID(0.5, 0, 0);*/
		//intakeHeight.enableControl();
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
	@Override
	public void autonomousInit() {
		ahrs.zeroYaw();
		autoTimer.reset();
		autoTimer.start();
	}
	
	double getMaxBotixValue(AnalogInput maxBotix){
		return maxBotix.getVoltage() / 0.0098;
	}
	
	boolean checkGoalLineReached(double tolerance){
		return getMaxBotixValue(MaxBotixY) <= (-131.18 / 163.13 * getMaxBotixValue(MaxBotixX) + 113.32) + tolerance && getMaxBotixValue(MaxBotixY) >= (-131.18 / 163.13 * getMaxBotixValue(MaxBotixX) + 113.32) - tolerance;
	}
	
	@Override
	public void autonomousPeriodic() {
		reportSensors();
		switch (AutoRoutine) {
		case BREACH : auto_BREACH(); break;
		//case AUTO_SCORE : auto_AUTO_SCORE(); break;
		case SIMPLE: auto_SIMPLE(); break;
		}	
	}
	
	void auto_SIMPLE(){
		if(autoTimer.get() < driveTime){
			PIDDrive(0.65);
		}
		else{
			PIDDrive(0);
		}
	}
	
	void auto_STRAIGHT () {
		PIDDrive(autoPower);
		if (!ahrs.isMoving() && autoPower < 1 && autoTimer.get() > 0.5) {
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
				AutonMode = AutonModeEnum.END;
			}
			break;

		case GO_BACK : 
			System.out.println("Going Back in Auto Breach");
			if(backupTimer.get() < backupTime){
				PIDDrive(-autoPower);
			}
			else {
				AutonMode = AutonModeEnum.GO_STRAIGHT;
				autoTimer.reset();
			}
			
		case END:
			PIDDrive(0);
			break;

		default: break;
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

		default:
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
	
	
	@Override
	public void teleopInit() {
		//camera.enable();
		ahrs.zeroYaw();
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//intakeHeight.set(10);
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
		if(level > 0 && !BallSensor.get() && !operatorStick.getRawButton(3)){
			level = 0;
		} else if (level > 0) {
			// scale down power for pulling the ball in but allow full power for output.
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
	@Override
	public void disabledInit(){
	//	camera.disable();
	}
	
	// This method looks for transitions on the auton button - points where the button changes from off to on
	boolean CheckAutonButton () {
		boolean lastValue = autoModeButtonDown;
		autoModeButtonDown = operatorStick.getRawButton(4);
		return (autoModeButtonDown && !lastValue);
	}
	
	@Override
	public void disabledPeriodic(){
		int idx;
		
		if (CheckAutonButton()) {	
			// Get current index
			idx = AutoRoutine.ordinal();
			
			// Get index of next choice with wraparound
			idx = (idx + 1) % (AutoRoutineEnum.values().length);
			
			AutoRoutine = AutoRoutineEnum.values()[idx];
			AutonMode = AutonModeEnum.GO_STRAIGHT;
		}

		SmartDashboard.putString("Auto Routine", AutoRoutine.name());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

	}
	
	public void reportSensors() {
		SmartDashboard.putBoolean(  "Ball Sensor",   BallSensor.get());
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		SmartDashboard.putString("Auto Mode", AutonMode.name());
		SmartDashboard.putNumber("Current Draw", PDP.getTotalCurrent());
		/*SmartDashboard.putNumber("Encoder position", ((int)Math.signum(intakeHeight.getPulseWidthPosition())) * Math.abs(intakeHeight.getPulseWidthPosition()) % 4096);
		SmartDashboard.putNumber("Relative Encoder Position", intakeHeight.getPosition());
		SmartDashboard.putNumber("Number of encoder rotations", ((int)Math.signum(intakeHeight.getPulseWidthPosition())) * Math.abs(intakeHeight.getPulseWidthPosition()) / 4096);
		SmartDashboard.putBoolean("TalonEnabled", intakeHeight.isEnabled());
		SmartDashboard.putBoolean("TalonControlEnabled", intakeHeight.isControlEnabled());
		SmartDashboard.putNumber("ClosedLoopError", intakeHeight.getClosedLoopError());*/
	}

}
