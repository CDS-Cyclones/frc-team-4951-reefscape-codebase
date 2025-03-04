package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

/**
 * This class is where all controllers are defined.
 */
public class OI {
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
    public static OperatorBoard m_operatorBoard = new OperatorBoard(OIConstants.kOperatorBoardPort);
}
