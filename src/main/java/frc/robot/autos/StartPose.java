package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum StartPose implements MutableChooser.Named {
  Outer("Subwoofer outer"),
  Inner("Subwoofer inner"),
  Front("Subwoofer front"),
  Wing("Wing");

  final String name;

  StartPose(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public Pose2d getPose() {
    switch (this) {
      case Outer:
        return new Pose2d(0.88, 6.57, Rotation2d.fromDegrees(60));
      case Inner:
        return new Pose2d(0.88, 4.53, Rotation2d.fromDegrees(-60));
      case Front:
        return new Pose2d(1.36, 5.55, new Rotation2d());
      default:
        // Arbitrary point within the starting zone very close to white tape line and opponent
        // source
        return new Pose2d(1.36, 1.62, new Rotation2d());
    }
  }
}
