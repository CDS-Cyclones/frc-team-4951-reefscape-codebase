package frc.robot;

import lombok.Getter;
import lombok.Setter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;

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


  public static enum DriveRotation {
    A(Constants.VisionConstants.aprilTagFieldLayout.getTagPose(18).get().getRotation().toRotation2d().minus(new Rotation2d(Math.PI))),
    B(Constants.VisionConstants.aprilTagFieldLayout.getTagPose(19).get().getRotation().toRotation2d().minus(new Rotation2d(Math.PI)));

    private final Rotation2d rotation2d;

    DriveRotation(Rotation2d rotation2d) {
      this.rotation2d = rotation2d;
    }

    public Rotation2d getRotation2d() {
      return rotation2d;
    }
  }

  private static DriveRotation driveRotation = DriveRotation.A;

  public static Rotation2d getDriveRotation2d() {
      return driveRotation.rotation2d;
  }

  public static void setDriveRotation(DriveRotation dR) {
      driveRotation = dR;
  }


}
