package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp
class TeleBasic : LinearOpMode() {
    override fun runOpMode() {
        val fr = hardwareMap.get(DcMotorEx::class.java, "rightFront")
        val fl = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        val br = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        val bl = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        waitForStart()
        while (!isStopRequested) {
            val leftY = gamepad1.left_stick_y
            val leftX = gamepad1.left_stick_x
            val rightY = gamepad1.right_stick_y
            val rightX = gamepad1.right_stick_x

            fl.power = (-rightX + leftX + leftY).toDouble()
            bl.power = (-rightX + (leftY-leftX)).toDouble()
            fr.power = (rightX + (leftY-leftX)).toDouble()
            br.power = (rightX + leftY + leftX).toDouble()
        }
    }
}
