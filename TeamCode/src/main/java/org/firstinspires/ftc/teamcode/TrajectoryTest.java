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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
//@Disabled
public class TrajectoryTest extends LinearOpMode {


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
    private AnalogInput Camera;


    //Functions
    private void OutakeOn() {
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setPower(0.65);
    }

    private void IntakeOff() {
        StarIntake.setPower(0);
    }
    private void MovingArmBack(ArmPositionBack ArmPosition) {
        switch(ArmPosition) {
            case Floor:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(0);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                break;
            case Bottom:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(380);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                break;
            case Middle:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(760);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                break;
            case Top:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(950);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                ArmMotor.setTargetPosition(2950);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case Middle:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(2700);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case Top:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(2375);
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

    private void SnapToAngle(double TargetAngle, double Timeout){
        ElapsedTime snapTime = new ElapsedTime();
        double HighSpeedBuffer = 10;
        double HighSpeed = 0.35;
        double LowSpeed = 0.25;
        int Turn = 180;
        double Angle;

        snapTime.reset();
        snapTime.startTime();

        if (Timeout == 0) Timeout = 9999;

        while ((snapTime.milliseconds() > Timeout) || !(Math.abs(Turn)<=0.5)) {
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


    private void MoveInches(double Inches, double Power, Direction direction,double Timeout) {
        double TargetDistance;
        ElapsedTime runTime = new ElapsedTime();
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
                runTime.reset();
                runTime.startTime();

                LeftFront.setPower(Power);
                LeftRear.setPower(Power);
                RightRear.setPower(Power);
                RightFront.setPower(Power);
                if (Timeout != 0) {
                    while ((runTime.milliseconds() < Timeout) || !(!LeftFront.isBusy() && !LeftRear.isBusy() && !RightFront.isBusy() && !RightRear.isBusy())) {
                        sleep(10);
                    }
                }
                else {
                    while (!(!LeftFront.isBusy() && !LeftRear.isBusy() && !RightFront.isBusy() && !RightRear.isBusy())) {
                        sleep(10);
                    }
                }

    }



    public void runOpMode() {
    //In Init

        //trajectory and road runner init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //creating position
        Pose2d startPose = new Pose2d(-41,-63,Math.toRadians(90));
        /*
        Pose2d Hub = new Pose2d(-15,-33,Math.toRadians(90));
        Pose2d Carousel = new Pose2d(-56,-50,Math.toRadians(25));
         */

        //telling localizer that this is where we are starting
        drive.setPoseEstimate(new Pose2d(-41,-63,Math.toRadians(90)));

        //building trajectories
        Trajectory TestTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-56,-50),Math.toRadians(45))
                .build();

        Trajectory ToCarousel = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-56,50), Math.toRadians(25))
                .build();
//-57,-58
        // .splineTo(new Vector2d(0,0),Math.toRadians(90))
        TrajectorySequence CarouselSequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-65,-56,Math.toRadians(90)))
                .build();

        TrajectorySequence Hub = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-12,-50),Math.toRadians(90))
                .build();

        TrajectorySequence GoToTop = drive.trajectorySequenceBuilder(Hub.end())
                //.lineToLinearHeading(new Pose2d (-12,-39, Math.toRadians(90)))
                .forward(8)
                .build();

        TrajectorySequence GoToMiddle = drive.trajectorySequenceBuilder(Hub.end())
                //.lineToLinearHeading(new Pose2d (-12,-39, Math.toRadians(90)))
                .forward(6)
                .build();

        TrajectorySequence GoToLow = drive.trajectorySequenceBuilder(Hub.end())
                //.lineToLinearHeading(new Pose2d (-12,-39, Math.toRadians(90)))
                .forward(6)
                .build();

        TrajectorySequence Warehouse = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d (10,-69,Math.toRadians(0)))
                .strafeRight(2)
                .lineToLinearHeading(new Pose2d (45,-69,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d (45,-48,Math.toRadians(90)))
                //.splineTo(new Vector2d(10,-40), Math.toRadians(0))
            //.splineTo(new Vector2d(24,-36), Math.toRadians(90))
                .build();

        TrajectorySequence Turn90 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
            .build();

        //Hardware Map
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rightCarouselSpinner = hardwareMap.get(DcMotor.class, "RightCarouselSpinner");
        LeftCarouselSpinner = hardwareMap.get(DcMotor.class, "LeftCarouselSpinner");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        StarIntake = hardwareMap.get(DcMotor.class, "StarIntake");
        Camera = hardwareMap.get(AnalogInput.class, "Camera");


        //IMU init
        //BNO055IMU.Parameters ImuParameters = new BNO055IMU.Parameters();
        //imu.initialize(ImuParameters);

        rightCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StarIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        


        //Directions
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        //In Play


        int HubPosition = 0;

        if (Camera.getVoltage() < 1.6) {
            //Left position is bottom
            HubPosition = 2;
        }
        if ((Camera.getVoltage() > 1.6) && (Camera.getVoltage() < 2.3)) {
            //Middle position is middle
            HubPosition = 1;
        }
        if (Camera.getVoltage() > 2.3) {
            //Right position is top
            HubPosition = 0;
        }

        telemetry.addData("Detected level", HubPosition);
        telemetry.update();

        drive.followTrajectorySequence(CarouselSequence);
        CarouselSpinnersON();
        sleep(2500);
        CarouselSpinnersOFF();
        drive.followTrajectorySequence(Hub);


        if (HubPosition == 0) {
            MovingArmFront(ArmPositionFront.Top);
            sleep(1000);
            drive.followTrajectorySequence(GoToTop);

        }
        if (HubPosition == 1) {
            MovingArmFront(ArmPositionFront.Middle);
            sleep(1500);
            drive.followTrajectorySequence(GoToMiddle);

        }
        if (HubPosition == 2) {
            MovingArmFront(ArmPositionFront.Bottom);
            sleep(1500);
            drive.followTrajectorySequence(GoToLow);
        }
        OutakeOn();
        sleep(1750);
        IntakeOff();

        drive.followTrajectorySequence(Warehouse);
        MovingArmBack(ArmPositionBack.Floor);
        sleep(2000);

    }
}
