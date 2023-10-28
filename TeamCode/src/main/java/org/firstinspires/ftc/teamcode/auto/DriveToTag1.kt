package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.movement.GAmovementKotlinRewrite
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Autonomous
class DriveToTag1 : LinearOpMode() {
    override fun runOpMode() {
        val camera = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        val portal = VisionPortal.easyCreateWithDefaults(camera, tagProcessor)
        var detections: List<AprilTagDetection?>
        val x = hardwareMap.get(ColorRangeSensor::class.java, "sensor")
        val move = mov(hardwareMap)
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
            move.update()
        }
    }
}

class mov(hardwareMap: HardwareMap) : GAmovementKotlinRewrite(
    hardwareMap.get(DcMotorEx::class.java, "rightFront"),
    hardwareMap.get(DcMotorEx::class.java, "leftFront"),
    hardwareMap.get(DcMotorEx::class.java, "rightRear"),
    hardwareMap.get(DcMotorEx::class.java, "leftRear"),
    hardwareMap,
    hardwareMap.get(DcMotorEx::class.java, "leftRear"),
    hardwareMap.get(DcMotorEx::class.java, "leftRear"),
    hardwareMap.get(IMU::class.java, "imu")
) {
    var hMap: HardwareMap = hardwareMap
    override fun autoConstructor (state:Int): Boolean {
        val camera = hMap.get(WebcamName::class.java, "Webcam 1")
        val tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        val portal = VisionPortal.easyCreateWithDefaults(camera, tagProcessor)
        var detections: List<AprilTagDetection?>
        val x = hMap.get(ColorRangeSensor::class.java, "sensor")
        detections = tagProcessor.detections
        var detect = detections[0]
        var dPose = detections[0]?.ftcPose
        var detected = false
        for (detection in detections) {
            val pose = detection.ftcPose
            if (detection.id == 1) {
                detect = detection
                dPose = pose
                detected = true
            }
            //STEPS
        }
        var tpi = 1800
        when (state) {
            1 -> if (detected && detect != null && dPose != null) {
                moveToTicks((dPose.range * tpi).toInt(), (dPose.bearing * tpi).toInt(), 0.5)
            }
        }
        return false
    }
}