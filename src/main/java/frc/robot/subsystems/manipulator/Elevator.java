package frc.robot.subsystems.manipulator;

import frc.chargers.utils.TunableValues.TunableNum;

public class Elevator {
    private static final TunableNum KP = new TunableNum("elevator/kP", 3000);
    private static final TunableNum KD = new TunableNum("elevator/kD", 350);
    private static final TunableNum DEMO_HEIGHT = new TunableNum("elevator/testHeight", 0);
}
