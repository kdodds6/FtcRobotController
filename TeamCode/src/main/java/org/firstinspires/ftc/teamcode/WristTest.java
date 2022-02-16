package org.firstinspires.ftc.teamcode;
//Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp

public class WristTest extends LinearOpMode {

    private Servo WristServo;

        public void runOpMode() {
            // In init

            WristServo = hardwareMap.get(Servo.class, "WristServo");


            waitForStart();
            // In Play

            //boolean used to activate and deactivate field centric at any moment


            while (opModeIsActive()) {

              double WristPosition = 0;
                boolean MoveWrist = true;

                if (MoveWrist) {
                    WristPosition = WristPosition + 0.5;
                    WristServo.setPosition(WristPosition);
                    telemetry.addData("Position", WristPosition);
                    telemetry.update();
                }

            }
        }
    }


    
