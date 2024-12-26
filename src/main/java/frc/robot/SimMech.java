package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SimMech {

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(120, 120);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 36, -90));

  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm", 23, Units.radiansToDegrees(0), 10, new Color8Bit(Color.kYellow)));

  // Create a Mechanism2d visualization of the elevator
  private final MechanismLigament2d m_elevator =
      m_arm.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(0), 0, 10, new Color8Bit(Color.kCyan)));

  // Create a Mechanism2d visualization of the wrist
  private final MechanismLigament2d m_wrist =
      m_elevator.append(
          new MechanismLigament2d(
              "Wrist", Units.metersToInches(.25), 0, 10, new Color8Bit(Color.kPurple)));

  public SimMech() {
    m_armTower.setColor(new Color8Bit(Color.kBlue));
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Simulation", m_mech2d);
  }

  public void setArm(double thetaRads) {
    // m_arm.setAngle(new Rotation2d(thetaRads));
    m_arm.setAngle(Units.radiansToDegrees(thetaRads));
  }

  public void setTelescope(double LengthM) {
    m_elevator.setLength(LengthM);
  }

  public void setWrist(double thetaRads) {
    // TODO: This should be a length. Need a theta --> length converer
    m_wrist.setAngle(new Rotation2d(thetaRads));
  }
}
