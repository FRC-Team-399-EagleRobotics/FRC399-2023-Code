import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.zeromq.ZContext;

import java.io.IOException;

public class AprilTagDetectionCommand extends CommandBase {
    private final NetworkTableInstance networkTable;
    private final VideoCapture videoCapture;
    private final ZContext zmqContext;
    private final AprilTagDetector detector;

    public AprilTagDetectionCommand(NetworkTableInstance networkTable) throws IOException {
        this.networkTable = networkTable;
        this.videoCapture = new VideoCapture(0);
        this.videoCapture.set(Videoio.CAP_PROP_FRAME_WIDTH, 320);
        this.videoCapture.set(Videoio.CAP_PROP_FRAME_HEIGHT, 240);
        this.zmqContext = new ZContext();
        this.detector = new AprilTagDetector();
        addRequirements(new AprilTagSubsystem());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AprilTag Detection Enabled", true);
    }

    @Override
    public void execute() {
        if (!SmartDashboard.getBoolean("AprilTag Detection Enabled", true)) {
            return;
        }

        Mat image = new Mat();
        if (videoCapture.read(image)) {
            Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2GRAY);
            TagDetectionArray detections = detector.detect(image);
            for (TagDetection detection : detections) {
                int id = detection.id;
                double x = detection.cxy.getX() / image.width();
                double y = detection.cxy.getY() / image.height();
                networkTable.getEntry("/vision/tag/" + id + "/x").setNumber(x);
                networkTable.getEntry("/vision/tag/" + id + "/y").setNumber(y);
            }
        }
        Timer.delay(0.01);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AprilTag Detection Enabled", false);
        videoCapture.release();
        zmqContext.close();
    }
}

class AprilTagSubsystem extends SubsystemBase {
    private final NetworkTableInstance networkTable;

    public AprilTagSubsystem() {
        this.networkTable = NetworkTableInstance.getDefault();
    }

    public double getTagX(int id) {
        return networkTable.getEntry("/vision/tag/" + id + "/x").getDouble(0.0);
    }

    public double getTagY(int id) {
        return networkTable.getEntry("/vision/tag/" + id + "/y").getDouble(0.0);
    }

    @Override
    public void periodic() {
        // do nothing
    }
}
