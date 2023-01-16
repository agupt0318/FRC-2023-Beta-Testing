package frc.lib.input;

import static frc.lib.input.ControllerConstants.XboxConstants.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Implementation of an Xbox controller
 */
public class XboxController implements Controller {
    private Joystick joystick;

    private Button aButton, bButton, xButton, yButton, leftMiddleButton, rightMiddleButton, leftBumper, rightBumper,
            leftStick, rightStick, leftTriggerButton, rightTriggerButton, rightStickYButton,
            upDpad, downDpad, leftDpad, rightDpad;

    public XboxController(Joystick joystick) {
        this.joystick = joystick;

        this.mapController();
    }

    public XboxController(int port) {
        this(new Joystick(port));
    }

    @Override
    public void mapController() {
        this.aButton = new JoystickButton(joystick, BUTTON_A);
        this.bButton = new JoystickButton(joystick, BUTTON_B);
        this.xButton = new JoystickButton(joystick, BUTTON_X);
        this.yButton = new JoystickButton(joystick, BUTTON_Y);
        this.leftMiddleButton = new JoystickButton(joystick, BUTTON_LEFT_MIDDLE);
        this.rightMiddleButton = new JoystickButton(joystick, BUTTON_RIGHT_MIDDLE);
        this.leftBumper = new JoystickButton(joystick, BUMPER_LEFT);
        this.rightBumper = new JoystickButton(joystick, BUMPER_RIGHT);
        this.leftStick = new JoystickButton(joystick, BUTTON_LEFT_STICK);
        this.rightStick = new JoystickButton(joystick, BUTTON_RIGHT_STICK);

    }

    /**
     * @return the value of the left trigger
     */
    public double getLeftTrigger() {
        return joystick.getRawAxis(JOYSTICK_LEFT_TRIGGER);
    }

    /**
     * @return the value of the right trigger
     */
    public double getRightTrigger() {
        return joystick.getRawAxis(JOYSTICK_RIGHT_TRIGGER);
    }

    /**
     * @return the value of the left joystick x-axis
     */
    public double getLeftX() {
        return joystick.getRawAxis(JOYSTICK_LEFT_X);
    }

    /**
     * @return the value of the left joystick y-axis (up is positive and down is
     *         negative)
     */
    public double getLeftY() {
        return -joystick.getRawAxis(JOYSTICK_LEFT_Y);
    }

    /**
     * @return the value of the right joystick x-axis
     */
    public double getRightX() {
        return joystick.getRawAxis(JOYSTICK_RIGHT_X);
    }

    /**
     * @return the value of the right joystick y-axis (up is positive and down is
     *         negative)
     */
    public double getRightY() {
        return -joystick.getRawAxis(JOYSTICK_RIGHT_Y);
    }

    /**
     * @return the A button
     */
    public Button getAButton() {
        return aButton;
    }

    /**
     * @return the B button
     */
    public Button getBButton() {
        return bButton;
    }

    /**
     * @return the X button
     */
    public Button getXButton() {
        return xButton;
    }

    /**
     * @return the Y button
     */
    public Button getYButton() {
        return yButton;
    }

    /**
     * @return the left middle button (start button? view button?)
     */
    public Button getLeftMiddleButton() {
        return leftMiddleButton;
    }

    /**
     * @return the right middle button (select button? menu button?)
     */
    public Button getRightMiddleButton() {
        return rightMiddleButton;
    }

    /**
     * @return the left bumper
     */
    public Button getLeftBumper() {
        return leftBumper;
    }

    /**
     * @return the right bumper
     */
    public Button getRightBumper() {
        return rightBumper;
    }

    /**
     * @return the left stick
     */
    public Button getLeftStick() {
        return leftStick;
    }

    /**
     * @return the right stick
     */
    public Button getRightStick() {
        return rightStick;
    }

    /**
     * @return the right trigger as a button
     */
    public Button getRightTriggerButton() {
        return rightTriggerButton;
    }

    /**
     * @return the left trigger as a button
     */
    public Button getLeftTriggerButton() {
        return leftTriggerButton;
    }

    /**
     * @return the right stick's y axis as a button
     */

    public Button getRightStickYButton() {
        return rightStickYButton;
    }

    /**
     * @return the up D-pad button
     */
    public Button getUpDpad() {
        return upDpad;
    }

    /**
     * @return the down D-pad button
     */
    public Button getDownDpad() {
        return downDpad;
    }

    /**
     * @return the left D-pad button
     */
    public Button getLeftDpad() {
        return leftDpad;
    }

    /**
     * @return the right D-pad button
     */
    public Button getRightDpad() {
        return rightDpad;
    }

    /**
     * Sets the rumble amount on the controller
     * 
     * @param type  Which rumble value to set
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    public void setRumble(RumbleType type, double value) {
        joystick.setRumble(type, value);
    }
}