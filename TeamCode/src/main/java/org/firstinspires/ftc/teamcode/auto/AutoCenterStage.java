package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

//import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.firstinspires.ftc.teamcode.movement.GAmovementKotlinRewrite;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class AutoCenterStage extends LinearOpMode {
    public boolean isLeft = false;
    public boolean isMiddle = false;
    public boolean isRight = false;

    @Override
    public void runOpMode() {

        CenterStageDetection detector = new CenterStageDetection();
        OpenCvCamera camera;

        isLeft = false;
        isMiddle = false;
        isRight = false;

        int width = 320;
        int height = 240;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();

        camera.setPipeline(detector);
        camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);
        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();

        telemetry.addData("Detecting Left: ", colorLeft);
        telemetry.addData("Detecting Center: ", colorMiddle);
        telemetry.update();

        //ENTER MOTOR NAMES AND STUFF IN HERE:
/*        Move m = new Move(

        );
 */
        //
        waitForStart();

        while (!isStopRequested()) {

            colorLeft = detector.getColorLeft();
            colorMiddle = detector.getColorMiddle();

            if ((colorLeft == CenterStageDetection.ColorDetected.CYAN) || (colorLeft == CenterStageDetection.ColorDetected.MAGENTA))
                isLeft = true;
            else if ((colorMiddle == CenterStageDetection.ColorDetected.CYAN) || (colorMiddle == CenterStageDetection.ColorDetected.MAGENTA))
                isMiddle = true;
            else
                isRight = true;


            telemetry.addData("Detecting Left: ", colorLeft);
            telemetry.addData("Detecting Center: ", colorMiddle);


    //
            //m.update();
            telemetry.update();
        }

//        if (isLeft) {
//            // Movements for left spot
//            telemetry.addData("Position", "Left");
//            telemetry.update();
//            sleep(20000);
//        }
//        else if (isMiddle) {
//            // Movements for center spot
//            telemetry.addData("Position", "Right");
//            telemetry.update();
//            sleep(20000);
//        }
//        else {
//            // Movements for right spot
//            telemetry.addData("Position", "Middle");
//            telemetry.update();
//            sleep(20000);
//        }

        camera.stopStreaming();
    }
}

class Move extends GAmovementKotlinRewrite {
    AutoCenterStage a;

    public Move(@NonNull DcMotorEx FrontLeftMotor, @NonNull DcMotorEx FrontRightMotor, @NonNull DcMotorEx BackLeftMotor, @NonNull DcMotorEx BackRightMotor, @Nullable HardwareMap hardwareMap, @NonNull DcMotorEx StrafeOdometer, @NonNull DcMotorEx DriveOdometer, @Nullable IMU Imu, AutoCenterStage mainClass) {
        super(FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor, hardwareMap, StrafeOdometer, DriveOdometer, Imu);
        a = mainClass;
    }
    int loc;

    @Override
    public void onStart() {
        if (a.isLeft) {
            loc = -1;
        } else if (a.isMiddle) {
            loc = 0;
        } else {
            loc = 1;
        }
    }
    @Override
    public boolean autoConstructor(int t) {
        switch (t) {
            case 1: //move to location
                if (loc == -1) {
                    moveToTicks(1892*7, 1892*3, 0.5);
                } else if (loc == 0) {
                    moveToTicks(1892*10, 0, 0.5);
                } else {
                    moveToTicks(1892*7, 0, 0.5);
                }
                break;
            case 2:
                moveClaw(0); //open una claw
                break;
            case 3: //park
                switch (loc) {
                    case -1:
                        moveToTicks(0, 1892 * 9, 0.5);
                        break;
                    case 0:
                        moveToTicks(0, 1892*10, 0.5);
                        break;
                    case 1:
                        moveToTicks(0, 1892*11, 0.5);
                        break;
                }
            default:
                break;
        }
        return true;
    }

    public void moveSlide(int number) {
        synchronize();
        if (true) {
            complete();
        }
    }
    public void moveClaw(int number) {
        synchronize();
        if (true) {
            complete();
        }
    }
}