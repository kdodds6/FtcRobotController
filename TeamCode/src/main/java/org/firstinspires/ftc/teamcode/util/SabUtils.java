package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SabUtils {
    public static DcMotor StarIntake;
    public static DcMotor rightCarouselSpinner;
    public static DcMotor LeftCarouselSpinner;
    public static DcMotor ArmMotor;
    public static AnalogInput Camera;

    public static enum ArmPositionBack {Floor,Bottom,Middle,Top}
    public static enum ArmPositionFront {Floor,Bottom,Middle,Top}

    public static void OutakeOn() {
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setPower(0.65);
    }

    public static void IntakeOff() {
        StarIntake.setPower(0);
    }

    public static void CarouselSpinnersON() {
        LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        rightCarouselSpinner.setPower(0.75);
        LeftCarouselSpinner.setPower(0.75);
    }

    public static void CarouselSpinnersOFF() {
        rightCarouselSpinner.setPower(0);
        LeftCarouselSpinner.setPower(0);
    }

    public static void MovingArmBack(ArmPositionBack ArmPosition) {
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
    public static void MovingArmFront(ArmPositionFront ArmPosition) {
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

}