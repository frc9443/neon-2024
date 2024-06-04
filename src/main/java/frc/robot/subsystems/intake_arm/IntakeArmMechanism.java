package frc.robot.subsystems.intake_arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeArmMechanism {
    private final Mechanism2d mechanism;
    private final MechanismLigament2d arm;
    private final String key;

    public IntakeArmMechanism(String key, Color color) {
        this.key = key;
        mechanism = new Mechanism2d(2.0, 2.0, new Color8Bit(Color.kBlanchedAlmond));
        MechanismRoot2d root = mechanism.getRoot("pivot", 1.5, 0.25);
        arm = new MechanismLigament2d("arm", 0.5, 42.0, 2, new Color8Bit(color));
        root.append(arm);
    }

    public void update(double radians) {
        arm.setAngle(Rotation2d.fromRadians(radians));
        Logger.recordOutput("IntakeArm/Mechanism2d/" + key, mechanism);
        Pose3d pose3d =  new Pose3d(0.0, 0.0, 0.4, new Rotation3d(0, -radians, 0));
        Logger.recordOutput("IntakeArm/Pose3d/" + key, pose3d);
    }
}
