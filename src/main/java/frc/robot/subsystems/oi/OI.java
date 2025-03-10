package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

/**
 * This class is where all controllers are defined.
 */
public class OI {
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static GenericHID m_operatorBoard = new GenericHID(OIConstants.kOperatorBoardPort);
    public static XboxController m_manipulatorController = new XboxController(OIConstants.kOperatorControllerPort);
    public static XboxController m_mainpulatorControllerManual = new XboxController(OIConstants.kOperatorControllerManualPort);
}
