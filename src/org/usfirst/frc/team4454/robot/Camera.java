package org.usfirst.frc.team4454.robot;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.CameraServer;

public class Camera extends Thread {
	int session;
	Image frame;
	public boolean status = false;
	final long delay = 10;
	
	public Camera(){
	}
	
	public synchronized void enable(){
		if(!status){
			frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
			// the camera name (ex "cam0") can be found through the roborio web interface
			session = NIVision.IMAQdxOpenCamera("cam0",
					NIVision.IMAQdxCameraControlMode.CameraControlModeController);
			NIVision.IMAQdxConfigureGrab(session);		
			NIVision.IMAQdxStartAcquisition(session);
			status = true;
		}
	}
	
	@Override
	public void run(){
		while(true) {
			update();
			try {
				sleep(delay);
			} catch (InterruptedException e) {
				System.out.println("Thread has been interrupted in sleep");
			}
		}
	}
	
	synchronized void update(){
		if(status){
			NIVision.IMAQdxGrab(session, frame, 1);
			CameraServer.getInstance().setImage(frame);
		}
	}
	public synchronized void disable(){
		if(status){
			NIVision.IMAQdxStopAcquisition(session);
			NIVision.IMAQdxCloseCamera(session);
			status = false;
		}
	}
}
