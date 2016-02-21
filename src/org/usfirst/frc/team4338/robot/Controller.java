/**
 * 
 */
package org.usfirst.frc.team4338.robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author Falcon Robotics
 *
 */
public class Controller {
	private DriverStation DS;
	private final int port;
	
	/**
	 * 
	 * @param port
	 */
	public Controller(int port){
		DS = DriverStation.getInstance();
		this.port = port;
	}
	
	/**
	 * 
	 * @param axis
	 * @return
	 */
	public double getRawAxis(int axis) {
		return DS.getStickAxis(port, axis);
	}
	
	/**
	 * 
	 * @param button
	 * @return
	 */
	public boolean getRawButton(int button) {
		return ((0x1 << (button - 1)) & DS.getStickButtons(port)) != 0;
	}
	
	/**
	 * 
	 * @return
	 */
	public int getPOV(){
		return DS.getStickPOV(port, 0);
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
		return getRawAxis(0);
	}
	
	public double getLeftJoyY() {
		return getRawAxis(1);
	}
	
	public double getLeftTrigger() {
		return getRawAxis(2);
	}
	
	public double getRightJoyX() {
		return getRawAxis(4);
	}
	
	public double getRightJoyY() {
		return getRawAxis(5);
	}
	
	public double getRightTrigger(){
		return getRawAxis(3);
	}
}
