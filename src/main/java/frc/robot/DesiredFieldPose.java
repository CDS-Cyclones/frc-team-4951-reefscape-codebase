package frc.robot;

import lombok.Getter;
import lombok.Setter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A class to store all the positions of the elevator and pivot.
 */
public final class DesiredFieldPose {
  // @Getter
  // @Setter
  // public static class MutableElevatorPosition {
  //   private double position;

  //   public MutableElevatorPosition(double position) {
  //     this.position = position;
  //   }

  //   public DoubleSupplier getSupplier() {
  //     return () -> position;
  //   }
  // }

  // public enum ElevatorPosition {
  //   DOWN(new MutableElevatorPosition(0.0)),
  //   L1(new MutableElevatorPosition(0.0)),
  //   L2(new MutableElevatorPosition(0.5)),
  //   L3(new MutableElevatorPosition(1.0)),
  //   L4(new MutableElevatorPosition(1.5)),
  //   BARGE(new MutableElevatorPosition(2.0));

  //   private final MutableElevatorPosition position;

  //   ElevatorPosition(MutableElevatorPosition position) {
  //     this.position = position;
  //   }

  //   public double getPosition() {
  //     return position.getPosition();
  //   }

  //   public void setPosition(double position) {
  //     this.position.setPosition(position);
  //   }

  //   public DoubleSupplier getSupplier() {
  //     return position.getSupplier();
  //   }
  // }

  // @Getter
  // @Setter
  // public static class MutablePivotPosition {
  //   private double position;

  //   public MutablePivotPosition(double position) {
  //     this.position = position;
  //   }

  //   public DoubleSupplier getSupplier() {
  //     return () -> position;
  //   }
  // }

  // public enum PivotPosition {
  //   IN(new MutablePivotPosition(0.588)),
  //   OUT(new MutablePivotPosition(.89));

  //   private final MutablePivotPosition position;

  //   PivotPosition(MutablePivotPosition position) {
  //     this.position = position;
  //   }

  //   public double getPosition() {
  //     return position.getPosition();
  //   }

  //   public void setPosition(double position) {
  //     this.position.setPosition(position);
  //   }

  //   public DoubleSupplier getSupplier() {
  //     return position.getSupplier();
  //   }
  // }

  // @Getter
  // @Setter
  // public static class MutableDriveRotation {
  //     private Rotation2d rotation2d;
  
  //     public MutableDriveRotation(Rotation2d rotation2d) {
  //         this.rotation2d = rotation2d;
  //     }
  
  //     public Supplier<Rotation2d> getSupplier() {
  //         return () -> rotation2d;
  //     }
  // }

  private static final Transform3d rotationToFaceTag = new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI));

  public static enum DrivePose {
    A(Constants.VisionConstants.aprilTagLayout.getTagPose(18).get().transformBy(rotationToFaceTag)),
    B(Constants.VisionConstants.aprilTagLayout.getTagPose(18).get().transformBy(rotationToFaceTag)),
    C(Constants.VisionConstants.aprilTagLayout.getTagPose(19).get().transformBy(rotationToFaceTag)),
    D(Constants.VisionConstants.aprilTagLayout.getTagPose(19).get().transformBy(rotationToFaceTag)),
    E(Constants.VisionConstants.aprilTagLayout.getTagPose(20).get().transformBy(rotationToFaceTag)),
    F(Constants.VisionConstants.aprilTagLayout.getTagPose(20).get().transformBy(rotationToFaceTag)),
    G(Constants.VisionConstants.aprilTagLayout.getTagPose(21).get().transformBy(rotationToFaceTag)),
    H(Constants.VisionConstants.aprilTagLayout.getTagPose(21).get().transformBy(rotationToFaceTag)),
    I(Constants.VisionConstants.aprilTagLayout.getTagPose(22).get().transformBy(rotationToFaceTag)),
    J(Constants.VisionConstants.aprilTagLayout.getTagPose(22).get().transformBy(rotationToFaceTag)),
    K(Constants.VisionConstants.aprilTagLayout.getTagPose(17).get().transformBy(rotationToFaceTag)),
    L(Constants.VisionConstants.aprilTagLayout.getTagPose(17).get().transformBy(rotationToFaceTag));

    private final Pose3d pose;

    DrivePose(Pose3d pose) {
      this.pose = pose;
    }

    public Rotation2d getRotation2d() {
      return pose.getRotation().toRotation2d();
    }

    public Pose3d getPose() {
      return pose;
    }
  }

  private static DrivePose drivePose = DrivePose.A;

  public static Rotation2d getDriveRotation2d() {
      return drivePose.getRotation2d();
  }

  public static Pose3d getDrivePose() {
      return drivePose.getPose();
  }

  public static void setDrivePose(DrivePose dP) {
      drivePose = dP;
  }
}
