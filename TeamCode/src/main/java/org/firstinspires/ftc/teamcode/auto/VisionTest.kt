package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

@Autonomous
class VisionTest: LinearOpMode() {
    override fun runOpMode() {
        val detector = CenterStageDetection()
        val camera: OpenCvCamera
        var isLeft = false
        var isMiddle = false
        var isRight = false
        val width = 320
        val height = 240
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance()
            .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)
        camera.openCameraDevice()
        camera.setPipeline(detector)

        camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT)
        waitForStart()
        var colorLeft = detector.colorLeft
        var colorMiddle = detector.colorMiddle
        camera.closeCameraDevice()
        val newCam = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        val portal: VisionPortal = VisionPortal.easyCreateWithDefaults(newCam, tagProcessor)
        var detections: List<AprilTagDetection?>
        while (!isStopRequested) {
            detections = tagProcessor.detections
            for (detection in detections) {
                val pose = detection.ftcPose
                telemetry.addLine("Detection ID: " + detection.id)
                telemetry.addLine("Distance: " + pose.range)
                telemetry.addLine("Bearing: " + pose.bearing)
                telemetry.addLine(
                    """
            Angle: ${pose.yaw}
            
            """.trimIndent()
                )
            }
            telemetry.update()
        }
        newCam.close()
    }
}