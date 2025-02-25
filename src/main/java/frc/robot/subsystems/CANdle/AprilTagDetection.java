package frc.robot.subsystems.CANdle;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.PoseRelToAprilTag;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
public class AprilTagDetection extends SubsystemBase {
    private final VisionSubsystem vision;
    private PhotonTrackedTarget latestTarget;
    private final CANdle candle;

    public AprilTagDetection(VisionSubsystem vision, SwerveSubsystem swerve, PoseRelToAprilTag desiredPose) {
        this.vision = vision;
        this.candle = new CANdle(40, "pigeonbus");
    }

    public boolean doesTagExist() {
        var photonResultOptional = vision.getCameras().get(0).getLatestResult();
        
        if (photonResultOptional.isPresent()) {
            var photonResult = photonResultOptional.get();
    
            if (photonResult.hasTargets()) {
                latestTarget = photonResult.getBestTarget();
                System.out.println("tag ID: " + latestTarget.getFiducialId());
                return true;
            }
        }
        return false;
    }
    public void lightUp() {
        candle.setLEDs(0, 255, 0);
    }
    public void lightDown() {
        candle.setLEDs(0,0,0);
    }
    public void checkLight(){
        if(doesTagExist()){
            lightUp();
        }
        else{
            lightDown();
        }
    }
@Override
public void periodic() {
    checkLight();
}
}