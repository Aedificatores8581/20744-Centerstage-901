package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.movement.GAmovementKotlinRewrite;
import org.firstinspires.ftc.teamcode.subsystems.colorDistance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        AprilTagProcessor tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(camera, tagProcessor);
        List<AprilTagDetection> detections;
        ColorRangeSensor x = hardwareMap.get(ColorRangeSensor.class, "sensor");
        colorDistance a = new colorDistance(x);
        TagMove i = new TagMove(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                hardwareMap.get(DcMotorEx.class, "rightRear"),
                null,
                //probably wrong:
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                null
        );
        while (!isStopRequested()) {
            detections = tagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                AprilTagPoseFtc pose = detection.ftcPose;
                telemetry.addLine("Detection ID: " + detection.id);
                telemetry.addLine("Distance: " + pose.range);
                telemetry.addLine("Bearing: " + pose.bearing);
                telemetry.addLine("Angle: " + pose.yaw + "\n");
                if (detection.id == 1) {
                    i.giveDetection(detection);
                }
            }
            telemetry.update();
            i.update();
        }
    }
}

class TagMove extends GAmovementKotlinRewrite {
    AprilTagDetection dets = null;
    AprilTagPoseFtc pose = null;

    public TagMove(@NonNull DcMotorEx FrontLeftMotor, @NonNull DcMotorEx FrontRightMotor, @NonNull DcMotorEx BackLeftMotor, @NonNull DcMotorEx BackRightMotor, @Nullable HardwareMap hardwareMap, @NonNull DcMotorEx StrafeOdometer, @NonNull DcMotorEx DriveOdometer, @Nullable IMU Imu) {
        super(FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor, hardwareMap, StrafeOdometer, DriveOdometer, Imu);
    }

    @Override
    public boolean autoConstructor(int t) {
        switch (t) {
            case 0:
                int tickY = 0;
                int tickX = 0;
                if (getTicks() == 0) {
                    if (pose.range >= 6) {
                        tickY = (int) pose.range * 1892;
                    }
                    if (pose.bearing >= 1) {
                        tickX = (int) pose.bearing * -1892;
                    }
                    if (pose.bearing <= -1) {
                        tickX = (int) pose.bearing * 1892;
                    }
                }
                moveToTicks(tickY, tickX, .5);
                break;
        }
        return true;
    }

    public void giveDetection(AprilTagDetection detection) {
        dets = detection;
        pose = detection.ftcPose;
    }
}