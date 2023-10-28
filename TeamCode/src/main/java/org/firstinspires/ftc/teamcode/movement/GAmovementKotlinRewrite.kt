package org.firstinspires.ftc.teamcode.movement

import com.google.gson.JsonParser
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import java.io.FileReader
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

abstract class GAmovementKotlinRewrite(
    FrontLeftMotor: DcMotorEx,
    FrontRightMotor: DcMotorEx,
    BackLeftMotor: DcMotorEx,
    BackRightMotor: DcMotorEx,
    hardwareMap: HardwareMap? = null,
    StrafeOdometer: DcMotorEx,
    DriveOdometer: DcMotorEx,
    Imu: IMU? = null) {
    private var fl: DcMotorEx = FrontLeftMotor
    private var fr: DcMotorEx = FrontRightMotor
    private var bl: DcMotorEx = BackLeftMotor
    private var br: DcMotorEx = BackRightMotor
    private var map: HardwareMap? = hardwareMap
    private var xOdo: DcMotorEx = StrafeOdometer
    private var yOdo: DcMotorEx = DriveOdometer
    private var imu: IMU? = Imu
    var gaTelemetryData = ""
    private val inch = 1892
    private var substate = 0
    private var substateGoal = 0
    private var ticks = 0
    private var state = 0
    private val accelExponent = 0.2
    private val decelExponent = 1.7
    private var endPosY = 0.0
    private var endPosX = 0.0

    //!!USE ADDITION!!//
    private val distToDecelY = intArrayOf(0, 100, 2000, 4900, 7400, 9800, 12800, 15700, 20900, 26600)
    private val distToDecelX = intArrayOf(0)
    private var speedY = 0.0
    private var speedX = 0.0
    private var deceleratingf = false
    private var deceleratings = false
    private var fnegative = 1
    private var snegative = 1
    private var stuckcountX = 0
    private var stuckdisttrackerX = 0
    private var stuckcountY = 0
    private var stuckdisttrackerY = 0

    //Rework vars
    private var targetY = 0
    private var targetX = 0
    private var targetO: Double = 0.0
    private var posThisStateX = 0
    private var posThisStateY = 0


    open fun autoConstructor(t: Int): Boolean {
        return true
    }

    public fun complete() {
        substate++
    }

    fun synchronize(): Boolean {
        if (ticks == 0) {
            substateGoal++
            return true
        }
        return false
    }

    fun update() {
        if (state == 0 && substate == 0) {
            onStart();
        }
        if (substate == substateGoal && ticks != 0) {
            state++
            ticks = 0
        }
        autoConstructor(state)
        ticks++
    }

    open fun onStart() {}

    fun moveToTicks(forward: Int, strafe: Int, targetSpeed: Double) {
        var newSpeedY: Double
        var newSpeedX: Double
        val Y = -yOdo.currentPosition
        val X = -xOdo.currentPosition
        if (forward < 0) {
            fnegative = -1
        }
        if (strafe < 0) {
            snegative = -1
        }
        newSpeedX = if (strafe == 0) {
            0.0
        } else {
            0.1 * snegative
        }
        newSpeedY = if (forward == 0) {
            0.0
        } else {
            0.1 * fnegative
        }
        if (synchronize()) {
            endPosY = (Y + forward).toDouble()
            endPosX = (X + strafe).toDouble()
            deceleratingf = false
            deceleratings = false
        } else {
            //forward math
            if (!deceleratingf) {
                newSpeedY = abs(speedY).pow(abs(accelExponent)).coerceAtMost(abs(targetSpeed)) * fnegative
                if (abs(endPosY - Y) <= distToDecelY[(newSpeedY * 10).toInt()]) {
                    deceleratingf = true
                }
            } else {
                newSpeedY = abs(speedY).pow(abs(decelExponent)).coerceAtLeast(0.0) * fnegative
            }
            //strafe math
            if (!deceleratings) {
                newSpeedX =
                    abs(speedX).pow(abs(accelExponent)).coerceAtMost(abs(targetSpeed)) * snegative
                if (abs(endPosX - X) <= distToDecelX[(newSpeedX * 10).toInt()]) {
                    deceleratings = true
                }
            } else {
                newSpeedX = abs(speedX).pow(abs(decelExponent)).coerceAtLeast(0.0) * snegative
            }
        }
        if (newSpeedX >= 1) {
            newSpeedX = 0.99
        }
        if (newSpeedX <= -1) {
            newSpeedX = -0.99
        }
        if (newSpeedY >= 1) {
            newSpeedY = 0.99
        }
        if (newSpeedY <= -1) {
            newSpeedY = -0.99
        }
        speedX = newSpeedX
        speedY = newSpeedY
        fl.power = speedY + speedX
        bl.power = speedY - speedX
        fr.power = speedY - speedX
        br.power = speedY + speedX
        if (newSpeedX == 0.0 && newSpeedY == 0.0) {
            complete()
        }
        gaTelemetryData = """
               Forward position: $Y
               Strafe Position: $X
               Forward speed: $speedY
               Strafe speed: $speedX
               Forward decel: $deceleratingf
               Strafe decel: $deceleratings
               Forward end: $endPosY
               Strafe end: 
               """.trimIndent() + endPosX
        distDataManager(speedY, Y)

        //obstacle failsafe:
        if (abs(X) - stuckdisttrackerX < 50 && forward <= 0.1) {
            stuckcountX++
        } else {
            stuckcountX = 0
        }
        if (stuckcountX >= 15) {
            deceleratingf = true
            gaTelemetryData = "Stuck (forward)!"
        }
        if (abs(Y) - stuckdisttrackerY < 50 && strafe <= 0.1) {
            stuckcountY++
        } else {
            stuckcountY = 0
        }
        if (stuckcountY >= 15) {
            deceleratings = true
            gaTelemetryData = "Stuck (strafe)!"
        }
        if (abs(Y) >= endPosY) {
            deceleratingf = true
        }
        if (abs(X) >= endPosX) {
            deceleratings = true
        }
    }
    private var prevSpeed = 0.0
    private var dists = ArrayList<Int>()
    var distData = ""

    var prevDist = 0
    private fun distDataManager(speed: Double, distance: Int) {
        prevDist = if (prevSpeed == speed) {
            dists.add(distance - prevDist)
            distance
        } else {
            dists.clear()
            0
        }
        prevSpeed = speed
        val average = AvgSpeedKotlinRewrite(dists.size, speed, dists.average())
        distData = """
               Speed: ${average.currentSpeed}
               Loop: ${average.loopsThisSpeed}
               Average distance for this speed: ${average.avgDistancePerLoop}
               """.trimIndent()
    }

    //WIP
    fun moveStandard (speed: Double) {
        if (imu == null) {
            gaTelemetryData += "!!WARNING!!\nCannot drive field centric with an undefined IMU"
            return
        }
        val yaw = imu!!.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
        val fTickVal = cos(yaw)*(targetX)+cos(yaw)*targetY
        val sTickVal = sin(yaw)*targetX+sin(yaw)*targetY
        if (!tolerate(yaw, targetO, 15.0)) {
            val turn = yaw - targetO
        }

    }
    fun setTarget(x:Int, y:Int, orientation:Double) {
        targetY = y
        targetX = x
        targetO = orientation
    }
    fun getTicks(): Int {
        return ticks;
    }

    private fun tolerate(value1:Double, value2:Double, tolerance:Double): Boolean {
        return abs(value1 - value2) < tolerance
    }
}

internal class AvgSpeedKotlinRewrite(
    var loopsThisSpeed: Int,
    var currentSpeed: Double,
    var avgDistancePerLoop: Double
)