//package org.firstinspires.ftc.teamcode.auto;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
////import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
//import org.firstinspires.ftc.teamcode.subsystems.PropPipeline;
//import org.firstinspires.ftc.teamcode.subsystems.Side;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//@Autonomous
//public class AutoCenterStage extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        boolean isLeft = false;
//        boolean isMiddle = false;
//        boolean isRight = false;
//
//        int width = 320;
//        int height = 240;
//
//        VisionPortal.Builder myVisionPortalBuilder;
//        VisionPortal myVisionPortal;
//
//// Create a new VisionPortal Builder object.
//        myVisionPortalBuilder = new VisionPortal.Builder();
//
//// Specify the camera to be used for this VisionPortal.
//        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));      // Other choices are: RC phone camera and "switchable camera name".
//
//// Add the AprilTag Processor to the VisionPortal Builder.
//
//        VisionProcessor myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
//        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);       // An added Processor is enabled by default.
//
//        VisionProcessor propPipeline = new PropPipeline();
//        myVisionPortalBuilder.addProcessor(propPipeline);
//
//// Optional: set other custom features of the VisionPortal (4 are shown here).
//        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
//        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
//        myVisionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).
//        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.
//
//// Create a VisionPortal by calling build()
//        myVisionPortal = myVisionPortalBuilder.build();
//
//        waitForStart();
//
//        while (!isStopRequested()) {
//
//            Side side = PropPipeline.getLocation();
//
//            telemetry.addData("Side: ", side);
//            telemetry.update();
//        }
//
////        if (isLeft) {
////            // Movements for left spot
////            telemetry.addData("Position", "Left");
////            telemetry.update();
////            sleep(3000);
//
////        }
////        else if (isMiddle) {
////            // Movements for center spot
////            telemetry.addData("Position", "Right");
////            telemetry.update();
////            sleep(20000);
////        }
////        else {
////            // Movements for right spot
////            telemetry.addData("Position", "Middle");
////            telemetry.update();
////            sleep(20000);
////        }
//
//        sleep(20000);
//
//         myVisionPortal.close();
//    }
//}
