package org.firstinspires.ftc.teamcode;
//Imports (Below)
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.reflect.Field;

 

@TeleOp

public class FieldCentricTest extends LinearOpMode {

    //declaring objects (Below)
    private BNO055IMU imu;


    //Functions (Below) (Get Heading)
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

    public void runOpMode() {
        // In init

        //Declaring Variables (Below)
        DcMotor LeftFront;
        DcMotor LeftRear;
        DcMotor RightFront;
        DcMotor RightRear;
        DcMotor Arm;
        DcMotor StarIntake;
        double JoystickAngle = 0;
        double Horizontal = 0;
        double Vertical = 0;
        double Piviot = 0;


        //Mapping Config objects to variables(Below)
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        StarIntake = hardwareMap.get(DcMotor.class, "StarIntake");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Directions (Below)
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        //Target Position(Below)
        Arm.setTargetPosition(0);
        StarIntake.setTargetPosition(0);


        //Modes (Below)
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        StarIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFront.setPower(0);
        RightRear.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);

        waitForStart();
        // In Play
        boolean FieldCentric = false;
        //boolean used to activate and deactivate field centric at any moment

        while (opModeIsActive()) {
            Vertical = -gamepad1.left_stick_y;
            Horizontal = gamepad1.left_stick_x;
            Piviot = gamepad1.right_stick_x;
            // Above, Getting Joystick Values (For both robot and field centric)

            if (!FieldCentric) {
                //Robot Centric
                RightFront.setPower(-Piviot + (Vertical - Horizontal));
                RightRear.setPower(-Piviot + Vertical + Horizontal);
                LeftFront.setPower(Piviot + Vertical + Horizontal);
                LeftRear.setPower(Piviot + (Vertical - Horizontal));
            }
            if (FieldCentric) {
                JoystickAngle = Math.atan2(Vertical, Horizontal) / Math.PI * 180;
                //Field Centric

                if (JoystickAngle < 0) {
                    JoystickAngle = JoystickAngle + 360;
                }
                //Above, deals with crossing O
                double DriveAngle = (JoystickAngle - GetHeading()) + 90;
                double Length = Math.sqrt(Math.pow(Horizontal, 2) + Math.pow(Vertical, 2));
                double ModdedVertical = Length * Math.sin(DriveAngle / 180 * Math.PI);
                double ModdedHorizontal = Length * Math.cos(DriveAngle / 180 * Math.PI);
                // Above, manipulating Joystick Angles to be Field Centric

                telemetry.addData("Horizontal", Horizontal);
                telemetry.addData("ModdedHorizontal", ModdedHorizontal);
                telemetry.update();
                //telemetry to test

                RightFront.setPower((-Piviot) + (ModdedVertical - ModdedHorizontal));
                RightRear.setPower((-Piviot) + (ModdedVertical + ModdedHorizontal));
                LeftFront.setPower(Piviot + (ModdedVertical + ModdedHorizontal));
                LeftRear.setPower(Piviot + (ModdedVertical - ModdedHorizontal));
                //setting powers
            }
            if (gamepad1.a) {
                //button a = arm to (NEED TO RE- MEASURE VALUES)
                Arm.setPower(0.5);
                Arm.setTargetPosition(0);
            } else if (gamepad1.b) {
                //button b = arm to
                Arm.setPower(0.75);
                Arm.setTargetPosition(2300);
            } else if (gamepad1.y) {
                //button y = arm to
                Arm.setPower(0.75);
                Arm.setTargetPosition(2577);
            } else if (gamepad1.x) {
                //button x = arm to
                Arm.setPower(0.75);
                Arm.setTargetPosition(2775);
            } else if (gamepad1.right_bumper) {
                //Right Bumper = turn off arm
                Arm.setPower(0);
            } else if (gamepad1.dpad_left) {
                //Dpad left = activates outake
                StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                StarIntake.setPower(0.5);
            } else if (gamepad1.dpad_right) {
                //Dpad right = activate intake
                StarIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                StarIntake.setPower(0.5);
            }
            telemetry.addData("Arm Position", Arm.getCurrentPosition());
            telemetry.update();
            //telemetry for de-bugging
        }
    }
}
    
