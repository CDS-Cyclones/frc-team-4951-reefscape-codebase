package frc.robot;

import lombok.Getter;
import lombok.Setter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A class to store all the positions of the elevator and pivot.
 */
public final class MutableFieldPose {
  private static final double inFrontOfTag = 0.1;
  private static final double inFrontOfTagSim = 0.5;
  private static final double leftOfTag = -0.3;
  private static final double rightOfTag = 0.3;

  /**
   * An enum to represent all desired field poses of the robot.
   */
  public static enum FieldPose {
    A(21, 10, inFrontOfTag, rightOfTag),
    B(21, 10, inFrontOfTag, leftOfTag),
    C(22, 9, inFrontOfTag, rightOfTag),
    D(22, 9, inFrontOfTag, leftOfTag),
    E(17, 8, inFrontOfTag, rightOfTag),
    F(17, 8, inFrontOfTag, leftOfTag),
    G(18, 7, inFrontOfTag, rightOfTag),
    H(18, 7, inFrontOfTag, leftOfTag),
    I(19, 6, inFrontOfTag, rightOfTag),
    J(19, 6, inFrontOfTag, leftOfTag),
    K(20, 11, inFrontOfTag, rightOfTag),
    L(20, 11, inFrontOfTag, leftOfTag);

    private final int tagBlueId;
    private final int tagRedId;
    private final double away;
    private final double side;

    FieldPose(int tagBlueId, int tagRedId, double away, double side) {
      this.tagBlueId = tagBlueId;
      this.tagRedId = tagRedId;
      this.away = away;
      this.side = side;
    }

    public int getTagId() {
      return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red  ? tagRedId : tagBlueId;
    }

    public Pose3d getTagPose() {
      return Constants.VisionConstants.aprilTagLayout.getTagPose(getTagId()).get();
    }

    public Pose3d getDesiredPose() {
      Pose3d tagPose = getTagPose();
      double tagAngle = tagPose.getRotation().toRotation2d().getRadians();
      double tagX = tagPose.getTranslation().getX();
      double tagY = tagPose.getTranslation().getY();

      // Ensure the angle is between 0 and 2pi
      if (tagAngle < 0) {
        tagAngle = 2 * Math.PI + tagAngle;
      }

      double cos = Math.cos(tagAngle);
      double sin = Math.sin(tagAngle);

      double newX = tagX;
      double newY = tagY;
      Pose3d newPose;
  
      switch (Constants.currentMode) {
        case SIM:
          newX += inFrontOfTagSim * cos;
          newY += inFrontOfTagSim * sin;
        default:
          newX += away * cos;
          newY += away * sin;
      }

      // now do tranomration to the left or right of the tag
      newX += side * -sin;
      newY += side * cos;

      newPose = new Pose3d(new Translation3d(newX, newY, 0), new Rotation3d(0, 0, tagAngle + Math.PI));

      return newPose;
    }

    public Rotation2d getDesiredRotation2d() {
      return getDesiredPose().getRotation().toRotation2d();
    }
  }

  @Getter
  @Setter
  private static FieldPose mutableFieldPose = FieldPose.A;

  /**
   * Returns the rotation to face the tag.
   * 
   * @return the {@link Rotation2d} to face the tag
   */
  public static Rotation2d getRotationToFaceTag() {
    return mutableFieldPose.getDesiredRotation2d();
  }

  /**
   * Returns the desired field pose.
   * 
   * @return the desired field pose as {@link Pose3d}
   */
  public static Pose3d getDesiredFieldPose() {
    return mutableFieldPose.getDesiredPose();
  }
}
