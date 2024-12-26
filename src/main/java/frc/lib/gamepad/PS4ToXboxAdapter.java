package frc.lib.gamepad;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class PS4ToXboxAdapter extends XboxController {
  private PS4Controller m_PS4Controller = null;

  /**
   * Adapts a PS4Controller object to be usable as an XboxController via polymorhphism
   *
   * <p>This is useful for testing when you have access to a PS4 controller but not an Xbox one.
   * There are many ways to achieve this effect, but this method guarantees that none of our code
   * can interfere with the base XboxController class, which is a nice safety
   *
   * @param port
   */
  public PS4ToXboxAdapter(int port) {
    // calls XboxController constructor, which is required
    super(port);

    // create the object to be adapted into an Xbox interface
    m_PS4Controller = new PS4Controller(port);
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  @Override
  public double getLeftX() {
    return m_PS4Controller.getLeftX();
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  @Override
  public double getRightX() {
    return m_PS4Controller.getRightX();
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  @Override
  public double getLeftY() {
    return m_PS4Controller.getLeftY();
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  @Override
  public double getRightY() {
    return m_PS4Controller.getRightY();
  }

  /**
   * Get the L2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
   * opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  @Override
  public double getLeftTriggerAxis() {
    return m_PS4Controller.getL2Axis();
  }

  /**
   * Get the R2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
   * opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  @Override
  public double getRightTriggerAxis() {
    return m_PS4Controller.getR2Axis();
  }

  /**
   * Read the value of the L1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftBumper() {
    return m_PS4Controller.getL1Button();
  }

  /**
   * Read the value of the R1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightBumper() {
    return m_PS4Controller.getR1Button();
  }

  /**
   * Whether the L1 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftBumperPressed() {
    return m_PS4Controller.getL1ButtonPressed();
  }

  /**
   * Whether the R1 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightBumperPressed() {
    return m_PS4Controller.getR1ButtonPressed();
  }

  /**
   * Whether the L1 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftBumperReleased() {
    return m_PS4Controller.getL1ButtonReleased();
  }

  /**
   * Whether the R1 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightBumperReleased() {
    return m_PS4Controller.getR1ButtonReleased();
  }

  /**
   * Read the value of the L3 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftStickButton() {
    return m_PS4Controller.getL3Button();
  }

  /**
   * Read the value of the R3 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightStickButton() {
    return m_PS4Controller.getR3Button();
  }

  /**
   * Whether the left stick button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftStickButtonPressed() {
    return m_PS4Controller.getL3ButtonPressed();
  }

  /**
   * Whether the R3 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightStickButtonPressed() {
    return m_PS4Controller.getR3ButtonPressed();
  }

  /**
   * Whether the L3 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftStickButtonReleased() {
    return m_PS4Controller.getL3ButtonReleased();
  }

  /**
   * Whether the R3 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightStickButtonReleased() {
    return m_PS4Controller.getR3ButtonReleased();
  }

  /**
   * Read the value of the Cross button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getAButton() {
    return m_PS4Controller.getCrossButton();
  }

  /**
   * Whether the Cross button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getAButtonPressed() {
    return m_PS4Controller.getCrossButtonPressed();
  }

  /**
   * Whether the Cross button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getAButtonReleased() {
    return m_PS4Controller.getCrossButtonReleased();
  }

  /**
   * Read the value of the Circle button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getBButton() {
    return m_PS4Controller.getCircleButton();
  }

  /**
   * Whether the Circle button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBButtonPressed() {
    return m_PS4Controller.getCircleButtonPressed();
  }

  /**
   * Whether the Circle button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getBButtonReleased() {
    return m_PS4Controller.getCircleButtonReleased();
  }

  /**
   * Read the value of the Square button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getXButton() {
    return m_PS4Controller.getSquareButton();
  }

  /**
   * Whether the Square button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getXButtonPressed() {
    return m_PS4Controller.getSquareButtonPressed();
  }

  /**
   * Whether the Square button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getXButtonReleased() {
    return m_PS4Controller.getSquareButtonReleased();
  }

  /**
   * Read the value of the Triange button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getYButton() {
    return m_PS4Controller.getTriangleButton();
  }

  /**
   * Whether the Triange button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getYButtonPressed() {
    return m_PS4Controller.getTriangleButtonPressed();
  }

  /**
   * Whether the Triange button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getYButtonReleased() {
    return m_PS4Controller.getTriangleButtonReleased();
  }

  /**
   * Read the value of the Share button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getBackButton() {
    return m_PS4Controller.getShareButton();
  }

  /**
   * Whether the Share button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBackButtonPressed() {
    return m_PS4Controller.getShareButtonPressed();
  }

  /**
   * Whether the Share button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getBackButtonReleased() {
    return m_PS4Controller.getShareButtonReleased();
  }

  /**
   * Read the value of the Options button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getStartButton() {
    return m_PS4Controller.getOptionsButton();
  }

  /**
   * Whether the Options button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getStartButtonPressed() {
    return m_PS4Controller.getOptionsButtonPressed();
  }

  /**
   * Whether the Options button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getStartButtonReleased() {
    return m_PS4Controller.getOptionsButtonReleased();
  }
}
