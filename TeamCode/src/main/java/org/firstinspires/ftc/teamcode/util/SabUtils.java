package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class SabUtils {
    public static DcMotor StarIntake;
    public static DcMotor rightCarouselSpinner;
    public static DcMotor LeftCarouselSpinner;
    public static DcMotor ArmMotor;
    public static AnalogInput Camera;
    public static Servo WristServo;

    public enum ArmPositionBack {Floor,Bottom,Middle,Top}
    public enum ArmPositionFront {Floor,Bottom,Middle,Top}
    public enum Alliance {Red,Blue}

    public static void OutakeOn() {
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setPower(0.65);
    }
    public static void IntakeOn() {
        StarIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        //IntakeServo.setDirection(Servo.Direction.REVERSE);
        StarIntake.setPower(0.75);
    }

    public static void IntakeOff() {
        StarIntake.setPower(0);
    }

    public static void CarouselSpinnersON() {
        LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        rightCarouselSpinner.setPower(0.5);
        LeftCarouselSpinner.setPower(0.5);
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

                WristServo.setDirection(Servo.Direction.REVERSE);
                WristServo.setPosition(0.3);
                break;
                /*
            case Bottom:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(4);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                WristServo.setDirection(Servo.Direction.REVERSE);
                WristServo.setPosition(0.05);
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

                 */

            default:
                throw new IllegalStateException("Unexpected value: " + ArmPosition);
        }
    }
    public static void MovingArmFront(ArmPositionFront ArmPosition) {
        switch(ArmPosition) {
            case Floor:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(3150);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                WristServo.setDirection(Servo.Direction.REVERSE);
                WristServo.setPosition(0.35);
                break;
            case Bottom:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(3000);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                WristServo.setDirection(Servo.Direction.REVERSE);
                WristServo.setPosition(0.5);
                break;
            case Middle:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(2550);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                WristServo.setDirection(Servo.Direction.REVERSE);
                WristServo.setPosition(0.4);
                break;
            case Top:
                ArmMotor.setPower(0.75);
                ArmMotor.setTargetPosition(2150);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                WristServo.setDirection(Servo.Direction.REVERSE);
                WristServo.setPosition(0.35);
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + ArmPosition);
        }
    }

    public static int CameraLevel(Alliance alliance) {

        int HubPosition = 999;

        if (alliance == Alliance.Red) {
            if (Camera.getVoltage() < 1.7) {
                //Left position is bottom
                HubPosition = 2;
            }
            if ((Camera.getVoltage() > 1.7) && (Camera.getVoltage() < 2.4)) {
                //Middle position is middle
                HubPosition = 1;
            }
            if (Camera.getVoltage() > 2.4) {
                //Right position is top
                HubPosition = 0;
            }
        }
        if(alliance == Alliance.Blue){
            if (Camera.getVoltage() < 1.5) {
                //Left position is bottom
                HubPosition = 2;
            }
            if ((Camera.getVoltage() > 1.5) && (Camera.getVoltage() < 0.9)) {
                //Middle position is middle
                HubPosition = 1;
            }
            if (Camera.getVoltage() > 0.9) {
                //Right position is top
                HubPosition = 0;
            }
        }
        return HubPosition;

    }

}