package frc.auton.guiauto.guiauto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.auton.guiauto.AbstractGuiAuto;

public class NetworkAuto extends AbstractGuiAuto {

    static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    static NetworkTable table = instance.getTable("autodata");
    static NetworkTableEntry autoPath = table.getEntry("autoPath");

    public NetworkAuto() {
        super(autoPath.getString(null));
    }

}