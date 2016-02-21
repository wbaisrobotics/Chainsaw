package org.usfirst.frc.team4338.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ParticleFilterCriteria2;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static final boolean B_ROBOT = false;
	
	private SpeedController arms;
	private AnalogPotentiometer armRope;
	private SpeedController chain;
	private RobotDrive drive;
	private Image frame;
	private Gyro gyro;
	
	private SpeedController leftRoller;
	private SpeedController rightRoller;
	private Joystick turnStick;
	private Joystick moveStick;
	
	private Encoder distanceEncoder;
	private AnalogInput rightIR;
	private AnalogInput leftIR;
	private int session;
	private Controller controller;
	// Default area minimum for particle as a percentage of total image area
	private double AREA_MINIMUM = 20;
	private NIVision.ParticleFilterCriteria2 criteria[] = new ParticleFilterCriteria2[1];
	private double leftRollerSpeed = 0.5;
	private double rightRollerSpeed = 0.5;
	
	private double turnSpeedScale = 1;
	private double moveSpeedScale = 1;
	
	private boolean leftDistanceCorrect = false;
	private boolean rightDistanceCorrect = false;
	
	private int leftDistanceFromCrate = 800;
	private int rightDistanceFromCrate = 900;
	
	private boolean armsOpen = true;
	private boolean autoPickCrate = false;
	private boolean cratePulled = false;
	
	public Robot() {
		super();
	}
	
	@Override
	public void autonomousInit() {
		gyro.reset();
	}
	
	@Override
	public void disabledInit() {
		NIVision.IMAQdxStopAcquisition(session);
	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		rightIR = new AnalogInput(2);
		leftIR = new AnalogInput(3);
		gyro = new Gyro(1);
		controller = new Controller(2);
		turnStick = new Joystick(0);
		moveStick = new Joystick(1);
		
		// When there were two robots, this let you choose the corresponding
		// speed controllers.
		if (B_ROBOT) {
			drive = new RobotDrive(1, 2, 3, 5);
			chain = new Victor(0);
			arms = new Victor(4);
			leftRoller = new Victor(6);
			rightRoller = new Victor(7);
		} else {
			drive = new RobotDrive(2, 3, 6, 7);
			chain = new Victor(4);
			arms = new Victor(1);
			leftRoller = new Victor(0);
			rightRoller = new Victor(5);
		}
		
		drive.setInvertedMotor(MotorType.kFrontLeft, true);
		drive.setInvertedMotor(MotorType.kRearLeft, true);
		// leftMagicPencilSensor = new DigitalInput(0);
		distanceEncoder = new Encoder(1, 0, true, Encoder.EncodingType.k4X);
		armRope = new AnalogPotentiometer(0, 720, -345);
		
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		
		// Opens up a session for the robot camera.
		// The camera name can be found in the web interface.
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM,
				100.0, 0, 0);
	}
	
	@Override
	public void teleopInit() {
		NIVision.IMAQdxStartAcquisition(session);
		drive.setSafetyEnabled(false);
		distanceEncoder.setDistancePerPulse(.021 * 1.513);
		distanceEncoder.reset();
		gyro.reset();
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//ams2015 printing joystick values for debugging
		//System.out.println("L" + turnStick.getX() + ", " + turnStick.getY());
		//System.out.println("R" + moveStick.getX() + ", " + moveStick.getY());
		
		//ams2015 printing IR values for debugging
		System.out.println("L:" + leftIR.getValue());
		System.out.println("R:" + rightIR.getValue());
		
		NIVision.IMAQdxGrab(session, frame, 1);
		CameraServer.getInstance().setImage(frame);
		
		SmartDashboard.putNumber("Right IR", rightIR.getValue());
		SmartDashboard.putNumber("Left IR", leftIR.getValue());
		SmartDashboard.putNumber("armRope", armRope.get());
		
		//NEW CONTROLS
		
		//Manual controls
		if(!autoPickCrate){
			//Manual chain control with triggers
			if(controller.getLeftTrigger() > 0){
				chain.set(1);
			} else if(controller.getRightTrigger() > 0){
				chain.set(-1);
			} else{
				chain.set(0);
			}
			//Manual arm control
			if(controller.getPOV() == 0){
				arms.set(0.75);
			} else if(controller.getPOV() == 180){
				arms.set(-0.75);
			} else{
				arms.set(0);
			}
			//Manual roller control
			if(controller.getLeftJoyY() > 0.2){
				leftRoller.set(leftRollerSpeed);
				arms.set(-0.75);
			} else if(controller.getLeftJoyY() < -0.2){
				leftRoller.set(-leftRollerSpeed);
				arms.set(-0.75);
			} else{
				leftRoller.set(0);
			}
			if(controller.getRightJoyY() > 0.2){
				rightRoller.set(-rightRollerSpeed);
				arms.set(-0.75);
			} else if(controller.getRightJoyY() < -0.2){
				rightRoller.set(rightRollerSpeed);
				arms.set(-0.75);
			} else{
				rightRoller.set(0);
			}
		}
		
		//Auto pick crate
		if(controller.getButtonA()){
			autoPickCrate = true;
		}
		
		//Retry left or right side lifting
		if(controller.getButtonLB()){
			
		} else if(controller.getButtonRB()){
			
		}
		
		//General cancel
		if(controller.getButtonY()){
			chain.set(0);
			leftRoller.set(0);
			rightRoller.set(0);
			arms.set(0);
			autoPickCrate = false;
			//temp
			cratePulled = false;
		}
		
		//Pull crate if enable to
		if(autoPickCrate){
			pickCrate();
		}
		
		//ams2015 When the turnStick trigger is pulled the turning speed is reduced to 50%
		turnSpeedScale = turnStick.getTrigger() ? 0.5 : 1;
		//ams2015 When the moveStick trigger is pulled the movement speed is reduced to 50%
		moveSpeedScale = moveStick.getTrigger() ? 0.5 : 1;
		//ams2015 this sends the robot how much to move and turn
		//drive.mecanumDrive_Cartesian(-moveStick.getX() * moveSpeedScale, -moveStick.getY() * moveSpeedScale, Math.abs(turnStick.getX()) > .2 ? -turnStick.getX() * turnSpeedScale : 0, 0);
	}
	
	public void pickCrate(){
		leftDistanceCorrect = false;
		rightDistanceCorrect = false;
		
		if(cratePulled){
			arms.set(0.75);
			if(armRope.get() < -190){
				arms.set(0);
				armsOpen = true;
			}
			chain.set(-1);
		} else{
			armsOpen = false;
			arms.set(-0.75);
			if(leftIR.getValue() < leftDistanceFromCrate){
				arms.set(-0.75);
				leftRoller.set(leftRollerSpeed);
				leftDistanceCorrect = false;
			} else{
				leftRoller.set(0);
				leftDistanceCorrect = true;
			}
			if(rightIR.getValue() < rightDistanceFromCrate){
				arms.set(-0.75);
				rightRoller.set(-rightRollerSpeed);
				rightDistanceCorrect = false;
			} else{
				rightRoller.set(0);
				rightDistanceCorrect = true;
			}
		}
		
		if(leftDistanceCorrect && rightDistanceCorrect){
			cratePulled = true;
		}
	}
	
}
