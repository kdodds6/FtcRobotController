package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.reflect.Field;

@TeleOp

public class FieldCentricTest extends LinearOpMode {

    //declaring objects (Below)
    private BNO055IMU imu;
    private DcMotor LeftFront;
    private DcMotor LeftRear;
    private DcMotor RightFront;
    private DcMotor RightRear;
    private DcMotor armMotor;
    private DcMotor starIntake;
    private DcMotor rightCarouselSpinner;
    private DcMotor LeftCarouselSpinner;
    enum Direction {Forward, Backward, Left, Right}
    private Servo IntakeServo;

    //Functions (Get Heading)
    private double GetHeading() {
        float IMUAngle;
        double ModdedAngle;

        IMUAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (IMUAngle < 0) {
            ModdedAngle = 360 + IMUAngle;
        } else {
            ModdedAngle = IMUAngle;
        }
        return ModdedAngle;
    }


    private void MoveInches(double Inches, double Power, Direction direction) {
        double TargetDistance;
        //Directions and TargetDistance for each Direction
        switch (direction) {
            case Forward:
                telemetry.addData("forward","forward");
                LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                TargetDistance = Inches;
                break;

            case Backward:
                telemetry.addData("back","back");
                LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
                // Multiply by -1 for opposite direction
                TargetDistance = Inches * -1;
                break;
            case Left:
                telemetry.addData("left","left");
                LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
                RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                //Multiply by 1.05 for correction value
                TargetDistance = Inches * 1.05;
                break;
            case Right:
                telemetry.addData("right","right");
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
        telemetry.addData("power",Power);
        telemetry.addData("TargetInches",TargetDistance);
        telemetry.update();

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
            // In init

            //Declaring Variables (Below)
            double JoystickAngle = 0;
            double Horizontal = 0;
            double Vertical = 0;
            double Pivot = 0;
            double ArmPower = 0.75;
            boolean ArmNotReset = false;

            //Mapping Config objects to variables(Below)
            LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
            LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
            RightFront = hardwareMap.get(DcMotor.class, "RightFront");
            RightRear = hardwareMap.get(DcMotor.class, "RightRear");
            armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
            starIntake = hardwareMap.get(DcMotor.class, "StarIntake");
            rightCarouselSpinner = hardwareMap.get(DcMotor.class, "RightCarouselSpinner");
            LeftCarouselSpinner = hardwareMap.get(DcMotor.class, "LeftCarouselSpinner");
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");

            //IMU init
            BNO055IMU.Parameters ImuParameters = new BNO055IMU.Parameters();
            imu.initialize(ImuParameters);

            //Directions
            LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            starIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
            rightCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);

            //Target Position
            armMotor.setTargetPosition(0);
            starIntake.setTargetPosition(0);

            //Modes
            LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            starIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            LeftFront.setPower(0);
            RightRear.setPower(0);
            RightFront.setPower(0);
            LeftRear.setPower(0);

            waitForStart();
            // In Play

            //boolean used to activate and deactivate field centric at any moment
            boolean FieldCentric = true;

            while (opModeIsActive()) {

                double IMUy = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
                double IMUz = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double IMUx = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

            /*
            telemetry.addData("IMU Y", IMUy);
            telemetry.addData("IMU Z", IMUz);
            telemetry.addData("IMU X", IMUx);
            telemetry.update();
             */

                //MoveInches(10.0,0.25, Direction.Forward);
                //sleep(2000);

                Vertical = -gamepad1.left_stick_y;
                Horizontal = gamepad1.left_stick_x;
                Pivot = gamepad1.right_stick_x;
                // Above, Getting Joystick Values (For both robot and field centric)

                if (!FieldCentric) {
                    //Robot Centric
                    RightFront.setPower(-Pivot + (Vertical - Horizontal));
                    RightRear.setPower(-Pivot + (Vertical + Horizontal));
                    LeftFront.setPower(Pivot + (Vertical + Horizontal));
                    LeftRear.setPower(Pivot + (Vertical - Horizontal));
                }
                if (FieldCentric) {
                    //Field Centric
                    JoystickAngle = Math.atan2(Vertical, Horizontal) / Math.PI * 180;

                    // deals with crossing O
                    if (JoystickAngle < 0) {
                        JoystickAngle = JoystickAngle + 360;
                    }

                    //manipulating Joystick Angles to be Field Centric
                    double GetHeading = GetHeading();
                    double DriveAngle = (JoystickAngle - GetHeading) + 180;
                    double Length = Math.sqrt(Math.pow(Horizontal, 2) + Math.pow(Vertical, 2));
                    double ModdedVertical = Length * Math.sin(DriveAngle / 180 * Math.PI);
                    double ModdedHorizontal = Length * Math.cos(DriveAngle / 180 * Math.PI);

                    //telemetry to test
                    telemetry.addData("IMU Y", IMUy);
                    telemetry.addData("IMU Z", IMUz);
                    telemetry.addData("IMU X", IMUx);
                    telemetry.addData("Horizontal", Horizontal);
                    telemetry.addData("Vertical", Vertical);
                    telemetry.addData("JoystickAngle", JoystickAngle);
                    telemetry.addData("GetHeading", GetHeading);
                    telemetry.addData("DriveAngle", DriveAngle);
                    telemetry.addData("ModdedVertical", ModdedVertical);
                    telemetry.update();

                    //setting powers
                    RightFront.setPower((-Pivot) - (ModdedVertical - ModdedHorizontal));
                    RightRear.setPower((-Pivot) - (ModdedVertical + ModdedHorizontal));
                    LeftFront.setPower(Pivot - (ModdedVertical + ModdedHorizontal));
                    LeftRear.setPower(Pivot - (ModdedVertical - ModdedHorizontal));
                }

                double RightTrigger1 = gamepad1.right_trigger;
                double RightTrigger2 = gamepad2.right_trigger;

                if ((gamepad1.dpad_down && (RightTrigger1 < 0.3)) || (gamepad2.dpad_down && (RightTrigger2 < 0.3))) {
                    //arm = Floor (Back)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(0);
                    telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                    telemetry.addData("down", 1);
                    telemetry.update();
                } else if ((gamepad1.dpad_right && (RightTrigger1 < 0.3)) || (gamepad2.dpad_right && (RightTrigger2 < 0.3))) {
                    //arm = 1st tier (Back)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(380);
                    telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                    telemetry.addData("right", 1);
                    telemetry.update();
                } else if ((gamepad1.dpad_left && (RightTrigger1 < 0.3)) || (gamepad2.dpad_left && (RightTrigger2 < 0.3))) {
                    //arm = 2nd tier (Back)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(760);
                    telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                    telemetry.addData("left", 1);
                    telemetry.update();
                } else if ((gamepad1.dpad_up && (RightTrigger1 < 0.3)) || (gamepad2.dpad_up && (RightTrigger2 < 0.3))) {
                    //arm= 3rd tier (Back)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(1000);
                    //760-1000
                    telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                    telemetry.addData("up", 1);
                    telemetry.update();
                } else if (gamepad1.dpad_down && (RightTrigger1 > 0.3) || (gamepad2.dpad_down && (RightTrigger2 > 0.3))) {
                    //arm = Floor (Front)
                    IntakeServo.setDirection(Servo.Direction.REVERSE);
                    IntakeServo.setPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(3290);
                } else if (gamepad1.dpad_right && (RightTrigger1 > 0.3) || (gamepad2.dpad_right && (RightTrigger2 > 0.3))) {
                    //arm = 1st tier (Front)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(3044);
                } else if (gamepad1.dpad_left && (RightTrigger1 > 0.3) || (gamepad2.dpad_left && (RightTrigger2 > 0.3))) {
                    //arm = 2nd tier (Front)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(2742);
                } else if (gamepad1.dpad_up && (RightTrigger1 > 0.3) || (gamepad2.dpad_up && (RightTrigger2 > 0.3))) {
                    //arm = 3rd tier (Front)
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ArmPower);
                    armMotor.setTargetPosition(2400);
                }

                //telemetry.addData("Arm Position", Arm.getCurrentPosition());
                //telemetry.addData("TRIGGER VALUE", RightTrigger);

                if (gamepad1.a || gamepad2.a) {
                    //A = activates intake
                    starIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                    IntakeServo.setDirection(Servo.Direction.REVERSE);
                    starIntake.setPower(0.75);
                    IntakeServo.setPosition(0);
                } else if (gamepad1.b || gamepad2.b) {
                    //B = activate outake
                    starIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                    starIntake.setPower(0.25);
                    IntakeServo.setPosition(0.25);
                } else if (gamepad1.x || gamepad2.x) {
                    //X = Turns off Intake
                    starIntake.setPower(0);
                }

                if (gamepad1.y) {
                    LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightCarouselSpinner.setPower(0.75);
                    LeftCarouselSpinner.setPower(0.75);
                } else {
                    rightCarouselSpinner.setPower(0);
                    LeftCarouselSpinner.setPower(0);
                }

                if(gamepad1.left_bumper) {
                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armMotor.setPower(0.1);
                    ArmNotReset = true;
                }
                if(gamepad1.right_bumper) {
                    armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armMotor.setPower(0.1);
                    ArmNotReset = true;
                }
                if(ArmNotReset && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
                    armMotor.setPower(0);
                    armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmNotReset = false;
                }
            }
        }
    }


    
