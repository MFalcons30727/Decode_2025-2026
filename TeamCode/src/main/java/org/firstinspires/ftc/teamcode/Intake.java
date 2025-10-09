package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake Test", group = "linearOpMode" )

public class Intake extends LinearOpMode {

    private DcMotor intake = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake.setPower(1);
            }
        }
    }
}
