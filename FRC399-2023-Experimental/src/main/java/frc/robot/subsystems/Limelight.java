package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {

    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
    }


    public void setLight(boolean state) {   
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(state ? 3 : 1);
    }

    public double getX() {
        tx = table.getEntry("tx");

        SmartDashboard.putNumber("x", tx.getDouble(0.0));
        return tx.getDouble(0.0);

    }

    public double getY() {
        ty = table.getEntry("ty");
        return ty.getDouble(0.0);        
    }
}
