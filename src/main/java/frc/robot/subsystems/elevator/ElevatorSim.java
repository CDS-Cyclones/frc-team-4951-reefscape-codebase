package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ManipulatorConstants.*;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorSim extends Elevator {
  private final DCMotor elevatorGearbox = DCMotor.getNEO(2);
  private final SparkMaxSim motorSim = new SparkMaxSim(super.motor, elevatorGearbox);
  private final edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
    elevatorGearbox,
    elevatorGearing,
    elevatorCarriageMass,
    elevatorDrumRadius,
    elevatorMinHeightMetres,
    elevatorMaxHeightMetres,
    true,
    elevatorMinHeightMetres,
    0.02,
    0.0
  );

  public ElevatorSim() {
    super();
  }

  public void simulationPeriodic() {
    //set input(voltage)
    elevatorSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    //update-every 20 milliseconds
    elevatorSim.update(0.02);

    motorSim.iterate((elevatorSim.getVelocityMetersPerSecond() / elevatorDistancePerRevolution) * 60,
                RoboRioSim.getVInVoltage(),
                0.020);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
