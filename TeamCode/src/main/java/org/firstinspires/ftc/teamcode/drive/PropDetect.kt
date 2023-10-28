package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetection
import org.opencv.core.Point
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

@TeleOp
class PropDetect : LinearOpMode() {
    override fun runOpMode() {
        // store as variable here so we can access the location
        val detector = SleeveDetection(100.0, 100.0, 100, 100)
        val phoneCam: OpenCvCamera

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        phoneCam = OpenCvCameraFactory.getInstance()
            .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)
        phoneCam.openCameraDevice()
        phoneCam.setPipeline(detector)
        phoneCam.startStreaming(240, 240, OpenCvCameraRotation.SIDEWAYS_LEFT)
        val location = detector.position
        telemetry.addData("location:", location)
    }
}