<<<<<<< Updated upstream
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//    @Autonomous(name = "Shoot Three Balls", group = "Autonomous")
//    public class ShootThreeBalls extends LinearOpMode {
//
//        private CRServo shoot = null;
//        private CRServo indexer1 = null;
//
//        @Override
//        public void runOpMode() {
//            // Initialize hardware
//            shoot = hardwareMap.get(Servo.class, "shoot");
//            indexer1 = hardwareMap.get(Servo.class, "indexer1");
//
//            // Wait for the start signal
//            waitForStart();
//
//            if (opModeIsActive()) {
//                // Start the shooter motor at 70% power
//                shoot.setPosition(0.7);
//
//                // Shoot three balls using the indexer
//                for (int i = 0; i < 3; i++) {
//                    // Move the indexer to push a ball into the shooter
//                    indexer1.setPosition(1.0); // Adjust position as needed
//                    sleep(500); // Wait for the ball to load
//
//                    // Reset the indexer to its original position
//                    indexer1.setPosition(0.0); // Adjust position as needed
//                    sleep(500); // Wait for the indexer to reset
//                }
//
//                // Stop the shooter motor
//                shoot.setPosition(0.0);
//            }
//        }
//    }
=======
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

    @Autonomous(name = "Shoot Three Balls", group = "Autonomous")
    public class ShootThreeBalls extends LinearOpMode {

        private CRServo shoot = null;
        private CRServo indexer1 = null;

        @Override
        public void runOpMode() {
            // Initialize hardware
            shoot = (CRServo) hardwareMap.get(Servo.class, "shoot");
            indexer1 = (CRServo) hardwareMap.get(Servo.class, "indexer1");

            // Wait for the start signal
            waitForStart();

            if (opModeIsActive()) {
                // Start the shooter motor at 70% power
                shoot.setPower(0.7);

                // Shoot three balls using the indexer
                for (int i = 0; i < 3; i++) {
                    // Move the indexer to push a ball into the shooter
                    indexer1.setPower(1.0); // Adjust position as needed
                    sleep(500); // Wait for the ball to load

                    // Reset the indexer to its original position
                    indexer1.setPower(0.0); // Adjust position as needed
                    sleep(500); // Wait for the indexer to reset
                }

                // Stop the shooter motor
                shoot.setPower(0.0);
            }
        }
    }
>>>>>>> Stashed changes
