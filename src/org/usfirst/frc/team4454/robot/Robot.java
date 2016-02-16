
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

	Joystick leftStick  = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);

	Talon frontLeftMotor   = new Talon(0);
	Talon rearLeftMotor  = new Talon(1);
	Talon frontRightMotor = new Talon(2);
	Talon rearRightMotor = new Talon(3);
	Talon intake      = new Talon(4);
	Talon intake2 = new Talon(5);

	RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	
	//camera variables
	int session;
	Image frame;

	AHRS ahrs;
	
	Timer autoTimer = new Timer();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

		// the camera name (ex "cam0") can be found through the roborio web interface
		session = NIVision.IMAQdxOpenCamera("cam0",
				NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);


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
		final double kP = 0.05;

		if (autoTimer.get() <= 4) {
			double yaw = ahrs.getYaw();
			double c = 0.5;       // motor power - common mode
			double d = kP * yaw;  // turning amount - differential mode
			double left = (c+d);
			double right = (c-d);
			
			drive.tankDrive(left, right);
			reportAHRS();
		} else {
			drive.tankDrive(0, 0);
		}
	}

	public void teleopInit() {		
		NIVision.IMAQdxStartAcquisition(session);
		ahrs.zeroYaw();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		NIVision.IMAQdxGrab(session, frame, 1);
		CameraServer.getInstance().setImage(frame);
		double scale = 0.5;

		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 1;
		} else {
			scale = 0.5;
		}
		drive.tankDrive(-leftStick.getY() * scale, -rightStick.getY() * scale); 
		intake.set(-operatorStick.getY() / 2);
		intake2.set(operatorStick.getY() / 2);
		reportAHRS();
	}

	/**
	 * This function is called when the disabled button is hit.
	 * You can use it to reset subsystems before shutting down.
	 */
	public void disabledInit(){
		NIVision.IMAQdxStopAcquisition(session);
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

	}

}
