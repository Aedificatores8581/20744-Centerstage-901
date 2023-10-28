package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

@TeleOp
class TeleFieldCentric : LinearOpMode() {
    override fun runOpMode() {
        val fr = hardwareMap.get(DcMotorEx::class.java, "rightFront")
        val fl = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        val br = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        val bl = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        val imu = hardwareMap.get(IMU::class.java, "imu")
        var imuParam = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            )
        )
        imu.resetDeviceConfigurationForOpMode()
        imu.initialize(imuParam)
        imu.resetYaw()
        waitForStart()
        while (!isStopRequested) run {
            val X: Double = gamepad1.left_stick_x.toDouble()
            val Y: Double = gamepad1.left_stick_y.toDouble()
            val pivot: Double = gamepad1.right_stick_x.toDouble()
            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            val xlocal = X * Math.cos(-heading) - Y * Math.sin(-heading)
            val ylocal = Y * Math.sin(-heading) + Y * Math.cos(-heading)

            fl.power = -pivot + ylocal + xlocal
            bl.power = -pivot + (ylocal - xlocal)
            fr.power = pivot + (ylocal - xlocal)
            br.power = pivot + ylocal + xlocal
        }
    }
}