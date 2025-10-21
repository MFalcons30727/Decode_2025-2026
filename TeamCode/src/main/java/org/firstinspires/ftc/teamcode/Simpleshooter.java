package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter", group = "linear OpMode"  )
@Disabled
public class Simpleshooter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor shootermotor = null;

    @Override public void runOpMode(){

        shootermotor = hardwareMap.get(DcMotor.class, "shootermotor");

        shootermotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                shootermotor.setPower(1);
            } else {
                shootermotor.setPower(0);
            }
        }
    }
}
