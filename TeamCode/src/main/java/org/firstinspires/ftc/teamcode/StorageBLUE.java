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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.SabUtils;

@Autonomous
//@Disabled
public class StorageBLUE extends LinearOpMode {

    public void runOpMode() {
    //In Init

        //trajectory and road runner init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //creating position
        Pose2d startPose = new Pose2d(-41,-63,Math.toRadians(90));

        //telling localizer that this is where we are starting
        drive.setPoseEstimate(new Pose2d(-41,-63,Math.toRadians(90)));

        //building trajectories
        Trajectory TestTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-56,-50),Math.toRadians(45))
                .build();

        Trajectory ToCarousel = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-56,50), Math.toRadians(25))
                .build();

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
                .build();

        TrajectorySequence Turn90 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
            .build();

        //Hardware Map
        SabUtils.rightCarouselSpinner = hardwareMap.get(DcMotor.class, "RightCarouselSpinner");
        SabUtils.LeftCarouselSpinner = hardwareMap.get(DcMotor.class, "LeftCarouselSpinner");
        SabUtils.ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        SabUtils.StarIntake = hardwareMap.get(DcMotor.class, "StarIntake");
        SabUtils.Camera = hardwareMap.get(AnalogInput.class, "Camera");

        SabUtils.rightCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SabUtils.LeftCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SabUtils.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SabUtils.StarIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Directions
        SabUtils.ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        SabUtils.StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        //In Play

        int HubPosition = 0;

        if (SabUtils.Camera.getVoltage() < 1.6) {
            //Left position is bottom
            HubPosition = 2;
        }
        if ((SabUtils.Camera.getVoltage() > 1.6) && (SabUtils.Camera.getVoltage() < 2.3)) {
            //Middle position is middle
            HubPosition = 1;
        }
        if (SabUtils.Camera.getVoltage() > 2.3) {
            //Right position is top
            HubPosition = 0;
        }

        telemetry.addData("Detected level", HubPosition);
        telemetry.update();

        drive.followTrajectorySequence(CarouselSequence);
        SabUtils.CarouselSpinnersON();
        sleep(2500);
        SabUtils.CarouselSpinnersOFF();
        drive.followTrajectorySequence(Hub);

        if (HubPosition == 0) {
            SabUtils.MovingArmFront(SabUtils.ArmPositionFront.Top);
            sleep(1000);
            drive.followTrajectorySequence(GoToTop);

        }
        if (HubPosition == 1) {
            SabUtils.MovingArmFront(SabUtils.ArmPositionFront.Middle);
            sleep(1500);
            drive.followTrajectorySequence(GoToMiddle);

        }
        if (HubPosition == 2) {
            SabUtils.MovingArmFront(SabUtils.ArmPositionFront. Bottom);
            sleep(1500);
            drive.followTrajectorySequence(GoToLow);
        }
        SabUtils.OutakeOn();
        sleep(1750);
        SabUtils.IntakeOff();

        drive.followTrajectorySequence(Warehouse);
        SabUtils.MovingArmBack(SabUtils.ArmPositionBack.Floor);
        sleep(2000);

    }
}
