package frc.robot.mutables;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/**
 * A mutable class to represent the desired field pose of the robot.
 */
public final class MutableFieldPose {
  private static final double inFrontOfTag = 0.05;
  private static final double inFrontOfTagSim = 0.4;
  private static final double leftOfTag = -0.2;
  private static final double rightOfTag = 0.2;

  /**
   * An enum to represent all desired field poses of the robot.
   */
  @RequiredArgsConstructor
  public static enum FieldPose {
    A(21, 10, inFrontOfTag, rightOfTag, Math.PI, false),
    B(21, 10, inFrontOfTag, leftOfTag, Math.PI, false),
    C(22, 9, inFrontOfTag, rightOfTag, Math.PI, false),
    D(22, 9, inFrontOfTag, leftOfTag, Math.PI, false),
    E(17, 8, inFrontOfTag, rightOfTag, Math.PI, false),
    F(17, 8, inFrontOfTag, leftOfTag, Math.PI, false),
    G(18, 7, inFrontOfTag, rightOfTag, Math.PI, false),
    H(18, 7, inFrontOfTag, leftOfTag, Math.PI, false),
    I(19, 6, inFrontOfTag, rightOfTag, Math.PI, false),
    J(19, 6, inFrontOfTag, leftOfTag, Math.PI, false),
    K(20, 11, inFrontOfTag, rightOfTag, Math.PI, false),
    L(20, 11, inFrontOfTag, leftOfTag, Math.PI, false),
    Z1(21, 10, inFrontOfTag, 0, Math.PI, false),
    Z2(22, 9, inFrontOfTag, 0, Math.PI, false),
    Z3(17, 8, inFrontOfTag, 0, Math.PI, false),
    Z4(18, 7, inFrontOfTag, 0, Math.PI, false),
    Z5(19, 6, inFrontOfTag, 0, Math.PI, false),
    Z6(20, 11, inFrontOfTag, 0, Math.PI, false),
    STATION_LEFT(13, 1, 0, 0, 0, true),
    STATION_RIGHT(12, 2, 0, 0, 0, true),
    BARGE_LEFT(14, 5, 0, 0, Math.PI, true),
    BARGE_RIGHT(15, 4, 0, 0, Math.PI, true),
    PROCESSOR(16, 3, 0, 0, Math.PI, true);

    private final int tagBlueId;
    private final int tagRedId;
    private final double away;
    private final double side;
    private final double rotation; // in radians
    private final boolean orientationOnly;

    /**
     * Return ID of the tag the pose is relative to.
     * @return The ID of the tag.
     */
    public int getTagId() {
      return DriverStation.getAlliance().get() == Alliance.Red ? tagRedId : tagBlueId;
    }

    /**
     * Return the pose of the tag the pose is relative to.
     * @return {@link Pose3d} of the tag.
     */
    public Pose3d getTagPose() {
      return Constants.VisionConstants.aprilTagLayout.getTagPose(getTagId()).get();
    }

    /**
     * Return the desired pose of the robot.
     * If orientationOnly is true, this will return null.
     * @return {@link Pose3d} of the robot.
     */
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

      // now do transformation to the left or right of the tag
      newX += side * -sin;
      newY += side * cos;

      newPose = new Pose3d(new Translation3d(newX, newY, 0), new Rotation3d(0, 0, tagAngle + rotation));

      return newPose;
    }

    /**
     * Return the desired pose of the robot.
     * If orientationOnly is true, this will return null.
     * @param ignoreForwards Whether to ignore the forwards distance.
     * @param ignoreSideways Whether to ignore the sideways distance.
     * @param ignoreRotation Whether to ignore the rotation.
     * @return {@link Pose3d} of the robot.
     */
    public Pose3d getDesiredPose(boolean ignoreForwards, boolean ignoreSideways, boolean ignoreRotation) {
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

      // now do transformation to the left or right of the tag
      newX += side * -sin;
      newY += side * cos;

      newPose = new Pose3d(new Translation3d(
        ignoreForwards ? tagX : newX,
        ignoreForwards ? tagY : newY,
        0
      ), new Rotation3d(
        0,
        0,
        ignoreRotation ? tagAngle : tagAngle + rotation
      ));

      return newPose;
    }

  
    /**
     * Return the desired rotation of the robot.
     * @return {@link Rotation2d} of the robot.
     */
    public Rotation2d getDesiredRotation2d() {
      return getDesiredPose().getRotation().toRotation2d();
    }

    /**
     * Return whether the desired pose is orientation only meaning the robot should only
     * rotate to the desired angle and not move.
     * 
     * @return True if the desired pose is orientation only.
     */
    public boolean isOrientationOnly() {
      return orientationOnly;
    }

    @Override
    public String toString() {
      return name();
    }
  }

  @Getter @Setter private static FieldPose mutableFieldPose = FieldPose.A;

  /**
   * Links reef spike poses with their corresponding zones.
   * 
   * @param currentPose The current pose whether it be a reef spike or a zone.
   * @param useZonePoses Whether the returned pose should be a zone pose.
   * @return
   */
  public static FieldPose getCorrespondingPose(FieldPose currentPose, boolean useZonePoses) {
    switch (currentPose) {
      case A:
        return useZonePoses ? FieldPose.Z1 : FieldPose.A;
      case B:
        return useZonePoses ? FieldPose.Z1 : FieldPose.B;
      case C:
        return useZonePoses ? FieldPose.Z2 : FieldPose.C;
      case D:
        return useZonePoses ? FieldPose.Z2 : FieldPose.D;
      case E:
        return useZonePoses ? FieldPose.Z3 : FieldPose.E;
      case F:
        return useZonePoses ? FieldPose.Z3 : FieldPose.F;
      case G:
        return useZonePoses ? FieldPose.Z4 : FieldPose.G;
      case H:
        return useZonePoses ? FieldPose.Z4 : FieldPose.H;
      case I:
        return useZonePoses ? FieldPose.Z5 : FieldPose.I;
      case J:
        return useZonePoses ? FieldPose.Z5 : FieldPose.J;
      case K:
        return useZonePoses ? FieldPose.Z6 : FieldPose.K;
      case L:
        return useZonePoses ? FieldPose.Z6 : FieldPose.L;
      case Z1:
        return useZonePoses ? FieldPose.Z1 : FieldPose.A;
      case Z2:
        return useZonePoses ? FieldPose.Z2 : FieldPose.C;
      case Z3:
        return useZonePoses ? FieldPose.Z3 : FieldPose.E;
      case Z4:
        return useZonePoses ? FieldPose.Z4 : FieldPose.G;
      case Z5:
        return useZonePoses ? FieldPose.Z5 : FieldPose.I;
      case Z6:
        return useZonePoses ? FieldPose.Z6 : FieldPose.K;
      default:
        return currentPose;
    }
  }
}