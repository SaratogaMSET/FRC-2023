package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
//import java.util.ArrayList;
//import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Helper class for creating a {@link Mechanism2d} and 3D component representation of an arm. */
public class ArmVisualizer {
  private final String logKey;
  private double shoulderLength = 0.445;
  //private double elbowLength = 0.3683;
  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechanismRoot;
  private final MechanismLigament2d fixedLigament;
  private final MechanismLigament2d shoulderLigament;
  private final MechanismLigament2d elbowLigament;

  public ArmVisualizer(String logKey, Color8Bit colorOverride) {
    this.logKey = logKey;
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Arm", 2 + 0, 0); //config.origin.getX()
    fixedLigament =
        mechanismRoot.append(
            new MechanismLigament2d(
                "Fixed", 0, 90, 6, new Color8Bit(Color.kBlack))); //config.origin.getY()
    shoulderLigament =
        fixedLigament.append(
            new MechanismLigament2d(
                "Shoulder",
                0.445, //config.shoulder().length()
                90,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kDarkBlue)));
    elbowLigament =
        shoulderLigament.append(
            new MechanismLigament2d(
                "Elbow",
                0.3683, //config.elbow().length()
                90,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kBlue)));
  }

  public void update(double shoulderAngle, double elbowAngle) {
    shoulderLigament.setAngle(Units.radiansToDegrees(shoulderAngle) - 90.0);
    elbowLigament.setAngle(Units.radiansToDegrees(elbowAngle));
    Logger.getInstance().recordOutput("Mechanism2d/" + logKey, mechanism);

    var shoulderPose =
        new Pose3d(
            0,
            0.0,
            0.0,
            new Rotation3d(0.0, -shoulderAngle, 0.0));
    var elbowPose =
        shoulderPose.transformBy(
            new Transform3d(
                new Translation3d(shoulderLength, 0.0, 0.0),
                new Rotation3d(0.0, -elbowAngle, 0.0)));
    Logger.getInstance().recordOutput("Mechanism3d/" + logKey, shoulderPose, elbowPose);
  }
}
