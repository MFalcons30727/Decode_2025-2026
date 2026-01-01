package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FieldCentricBlueTeleOp", group = "TeleOp")
public class TestPoses extends OpMode {

    private DriverDanny driver;

    public Pose testPoses;

    @Override
    public void init() {
        driver = new DriverDanny(hardwareMap,
                telemetry,
                DriverDanny.Alliance.BLUE,
                DriverDanny.Poses.BLUE_FAR_START_POSE);

//        testPoses = new Pose[] {
//                DriverDanny.Poses.BLUE_FAR_START_POSE,
//                DriverDanny.Poses.BLUE_GOAL_POSE,
//                DriverDanny.Poses.PARK_RED_GATE_POSE,
//                DriverDanny.Poses.RED_GOAL_POSE,
//                DriverDanny.Poses.RED_FAR_START_POSE,
//                DriverDanny.Poses.BLUE_BACKUP_START_POSE,
//                DriverDanny.Poses.BLUE_FAR_SHOOTING_POSE,
//                DriverDanny.Poses.BLUE_FINAL_PARK_POSE,
//                DriverDanny.Poses.BLUE_NEAR_SHOOTING_POSE,
//                DriverDanny.Poses.PARK_BLUE_GATE_POSE,
//                DriverDanny.Poses.RED_BACKUP_START_POSE,
//                DriverDanny.Poses.RED_FAR_SHOOTING_POSE,
//                DriverDanny.Poses.RED_FINAL_PARK_POSE,
//                DriverDanny.Poses.RED_NEAR_SHOOTING_POSE
//        };
    }

    @Override
    public void loop() {
        driver.update();

    }

}