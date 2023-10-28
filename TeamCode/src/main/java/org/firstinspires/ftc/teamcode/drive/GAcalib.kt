package org.firstinspires.ftc.teamcode.drive

import android.os.Build
import androidx.annotation.RequiresApi
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.movement.GAmovementKotlinRewrite
import java.io.File

@TeleOp
@RequiresApi(Build.VERSION_CODES.O)
class GAcalib: LinearOpMode() {
    private val file: File = kotlin.io.path.createTempFile("GAcalib", "json").toFile()
    var a = AppUtil.getInstance().createTempFile("GAcalib", "json", file)
    override fun runOpMode() {
        a.setWritable(true)
        if (a.exists() && a.canWrite()) {

        }
    }
}