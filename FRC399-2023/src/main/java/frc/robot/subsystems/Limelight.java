package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable table;
    // Basic targeting
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    // AprilTag and 3D Data
    NetworkTableEntry tid;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        // Basic targeting
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        // AprilTag and 3D Data
        tid = table.getEntry("tid");
    }


    public void setLight(boolean state) {   
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(state ? 3 : 1);
    }

    // Begin of Basic targeting

    public double getX() {
        tx = table.getEntry("tx");

        SmartDashboard.putNumber("x", tx.getDouble(0.0));
        return tx.getDouble(0.0);

    }

    public double getY() {
        ty = table.getEntry("ty");
        return ty.getDouble(0.0);        
    }

    public double getA() {
        ta = table.getEntry("ta");
        return ta.getDouble(0.0);  
    }

    // End of Basic targeting

    // Begin of AprilTag and 3D Data
    // End of AprilTag and 3D Data
}
