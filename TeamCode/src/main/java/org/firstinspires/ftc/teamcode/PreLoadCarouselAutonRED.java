/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
//@Disabled
public class PreLoadCarouselAutonRED extends LinearOpMode {


    //Declaring Objects and enums
    private DcMotor LeftFront;
    private DcMotor LeftRear;
    private DcMotor RightFront;
    private DcMotor RightRear;
    private BNO055IMU imu;
    enum Direction {Forward, Backward, Left, Right}
    private DcMotor rightCarouselSpinner;
    private DcMotor LeftCarouselSpinner;
    enum ArmPositionBack {Floor,Bottom,Middle,Top}
    enum ArmPositionFront {Floor,Bottom,Middle,Top}
    enum IntakeState {Outake,Intake,Off}
    private DcMotor ArmMotor;
    private DcMotor StarIntake;


    //Functions
    private void OutakeOn() {
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setPower(0.3);
    }
    private void IntakeOff() {
        StarIntake.setPower(0);
    }
    private void MovingArmBack(ArmPositionBack ArmPosition) {
        switch(ArmPosition) {
            case Floor:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(0);
                break;
            case Bottom:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(380);
                break;
            case Middle:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(760);
                break;
            case Top:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(950);
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + ArmPosition);
        }
    }
    private void MovingArmFront(ArmPositionFront ArmPosition) {
        switch(ArmPosition) {
            case Floor:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(3290);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case Bottom:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(3044);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case Middle:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(2742);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case Top:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(2493);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + ArmPosition);
        }
    }

    private void CarouselSpinnersON() {
        LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        rightCarouselSpinner.setPower(0.75);
        LeftCarouselSpinner.setPower(0.75);
    }
    private void CarouselSpinnersOFF() {
        rightCarouselSpinner.setPower(0);
        LeftCarouselSpinner.setPower(0);
    }
    private double GetHeading() {
        float IMUAngle;
        float ModdedAngle;

        IMUAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (IMUAngle < 0) {
            ModdedAngle = 360 + IMUAngle;
        } else {
            ModdedAngle = IMUAngle;
        }
        telemetry.addData("GetHeading", ModdedAngle);
        return ModdedAngle;
    }

    private void SnapToAngle(double TargetAngle){
        double HighSpeedBuffer = 10;
        double HighSpeed = 0.35;
        double LowSpeed = 0.25;
        int Turn = 180;
        double Angle;

        while (!(Math.abs(Turn)<=0.5)) {
            Angle = TargetAngle - GetHeading();
            telemetry.addData("Difference to target:", Angle);
            if (Angle > 180) {
                Turn = (int) (Angle - 360);
            } else if (Angle < -180) {
                Turn = (int) (Angle + 360);
            } else if (Angle < 180 && Angle > -180) {
                Turn = (int) Angle;
            }
            telemetry.addData("Amount to turn:", Turn);

            if (Turn < 0) {
                // Turn
                LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftRear.setDirection(DcMotorSimple.Direction.FORWARD);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (Math.abs(Turn) > HighSpeedBuffer) {
                LeftFront.setPower(HighSpeed);
                LeftRear.setPower(HighSpeed);
                RightFront.setPower(HighSpeed);
                RightRear.setPower(HighSpeed);
            } else if (Math.abs(Turn) >= 0.5) {
                LeftFront.setPower(LowSpeed);
                LeftRear.setPower(LowSpeed);
                RightFront.setPower(LowSpeed);
                RightRear.setPower(LowSpeed);
                telemetry.addData("Slow","Turn");

            }
            telemetry.update();
        }
        LeftFront.setPower(0);
        LeftRear.setPower(0);
        RightFront.setPower(0);
        RightRear.setPower(0);
    }


    private void MoveInches(double Inches, double Power, Direction direction) {
        double TargetDistance;
        //Directions and TargetDistance for each Direction
        switch (direction) {
            case Forward:
                telemetry.addData("forward", "forward");
                LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                TargetDistance = Inches;
                break;

            case Backward:
                telemetry.addData("back", "back");
                LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                // Multiply by -1 for opposite direction
                TargetDistance = Inches * -1;
                break;
            case Left:
                telemetry.addData("left", "left");
                LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                //Multiply by 1.05 for correction value
                TargetDistance = Inches * 1.05;
                break;
            case Right:
                telemetry.addData("right", "right");
                LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                //Multiply by both -1 for directions and 1.05 for correction
                TargetDistance = (Inches * -1) * 1.05;
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + direction);

            }

                /*telemetry.addData("power", Power);
                telemetry.addData("TargetInches", TargetDistance);
                telemetry.update();
                 */

                //Calculations
                //Wheel diameter = 3.78 inches
                //537.7 = Encoder Counts per Revolution
                //Calculating Circumference (d * pi)
                double Circumference = 3.78 * 3.14;
                //(Encoder Counts per Revolution * Gear ratio (1:1)) / Circumference
                double EncoderCountsPerInch = (537.7 * 1) / Circumference;
                double FinalEncoderCounts = EncoderCountsPerInch * TargetDistance;
                //Modes
                LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //Target Positions
                LeftFront.setTargetPosition((int) FinalEncoderCounts);
                LeftRear.setTargetPosition((int) FinalEncoderCounts);
                RightRear.setTargetPosition((int) FinalEncoderCounts);
                RightFront.setTargetPosition((int) FinalEncoderCounts);
                //More Modes
                LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Power
                LeftFront.setPower(Power);
                LeftRear.setPower(Power);
                RightRear.setPower(Power);
                RightFront.setPower(Power);

                while (!(!LeftFront.isBusy() && !LeftRear.isBusy() && !RightFront.isBusy() && !RightRear.isBusy())) {
                    sleep(100);
                }
        }


    public void runOpMode() {
    //In Init

        //Hardware Maps
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rightCarouselSpinner = hardwareMap.get(DcMotor.class, "RightCarouselSpinner");
        LeftCarouselSpinner = hardwareMap.get(DcMotor.class, "LeftCarouselSpinner");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        StarIntake = hardwareMap.get(DcMotor.class, "StarIntake");
        
        //IMU init
        BNO055IMU.Parameters ImuParameters = new BNO055IMU.Parameters();
        imu.initialize(ImuParameters);

        rightCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StarIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        


        //Directions
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        //In Play

        String HubPosition = "3rd";

        //getting to carousel
        MoveInches(4,0.4,Direction.Forward);
        MoveInches(25,0.4,Direction.Left);
        //at carousel
        CarouselSpinnersON();
        sleep(3500);
        CarouselSpinnersOFF();
        //lined up at hub
        MoveInches(48,0.4,Direction.Right);
        if (HubPosition == "3rd") {
            //getting closer to hub
            MoveInches(12,0.4,Direction.Forward);
            MovingArmFront(ArmPositionFront.Top);
            sleep(3000);
        }
        if (HubPosition == "2nd") {
            MoveInches(7,0.4,Direction.Forward);
            MovingArmFront(ArmPositionFront.Middle);
            sleep(3000);
        }
        if (HubPosition == "1st") {
            MoveInches(13,0.4,Direction.Forward);
            MovingArmFront(ArmPositionFront.Bottom);
            sleep(3000);
        }

        //dropping off block
        OutakeOn();
        sleep(1750);
        IntakeOff();
        //getting arm in postion for tele-op
        //going to warehouse
        //MoveInches(10,0.4,Direction.Backward);
        SnapToAngle(270);
        //MoveInches(10,0.4,Direction.Right);
        MoveInches(70,0.8,Direction.Forward);
        SnapToAngle(0);
        MovingArmBack(ArmPositionBack.Floor);
        sleep(2000);





    }
}
