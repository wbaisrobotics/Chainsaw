package org.usfirst.frc.team4338.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class XBoxControllerOld {
	
	private DriverStation myDS;
	private final int myPort;
	
	/**
	 * Creates a new controller at the given port.
	 * 
	 * @param port
	 *            the port number of the controller
	 */
	public XBoxControllerOld(int port) {
		myDS = DriverStation.getInstance();
		myPort = port;
	}
	
	public boolean getButtonA() {
		return getRawButton(1);
	}
	
	public boolean getButtonB() {
		return getRawButton(2);
	}
	
	public boolean getButtonBack() {
		return getRawButton(7);
	}
	
	public boolean getButtonLB() {
		return getRawButton(5);
	}
	
	public boolean getButtonLS() {
		return getRawButton(9);
	}
	
	public boolean getButtonRB() {
		return getRawButton(6);
	}
	
	public boolean getButtonRS() {
		return getRawButton(10);
	}
	
	public boolean getButtonStart() {
		return getRawButton(8);
	}
	
	public boolean getButtonX() {
		return getRawButton(3);
	}
	
	public boolean getButtonY() {
		return getRawButton(4);
	}
	
	public double getLeftJoyX() {
		return getRawAxis(1);
	}
	
	public double getLeftJoyY() {
		return getRawAxis(2);
	}
	
	public double getLeftTrigger() {
		return Math.max(getRawAxis(3), 0);
	}
	
	public int getPov() {
		return myDS.getStickPOV(myPort, 0);
	}
	
	public double getRawAxis(int axis) {
		return myDS.getStickAxis(myPort, axis);
	}
	
	public boolean getRawButton(int button) {
		return ((0x1 << (button - 1)) & myDS.getStickButtons(myPort)) != 0;
	}
	
	public double getRightJoyX() {
		return getRawAxis(4);
	}
	
	public double getRightJoyY() {
		return getRawAxis(5);
	}
	
	/**
	 * Warning! getRightTrigger() and getLeftTrigger() both use getRawAxis(3).
	 * As getRawAxis(3) goes below zero, getRightTrigger() increases, and as
	 * getRawAxis(3) goes above zero, getLeftTrigger() increases. If both
	 * triggers are pressed, both of them will be treated as zero. You can only
	 * use one trigger at a time.
	 *
	 * @return
	 */
	public double getRightTrigger() {
		return -Math.min(getRawAxis(3), 0);
	}
}
