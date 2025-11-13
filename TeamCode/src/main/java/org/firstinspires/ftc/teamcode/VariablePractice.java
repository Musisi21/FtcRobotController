package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
public class VariablePractice {
    @TeleOp(name = "Variable Practice")
    public class VariablePractice extends OpMode {

        @Override
        public void init() {
            int teamNumber = 23814;
            double motorSpeed = 0.75;
            boolean clawClosed = true;
            String teamName = "The Flying Dutchman";
            int motorAngle = 98;

            telemetry.addData("Team Number", teamNumber);
            telemetry.addData("Motor Speed", motorSpeed);
            telemetry.addData("Claw Closed", clawClosed);
            telemetry.addData("Team Name", teamName);
            telemetry.addData("Motor Angle", motorAngle);
        }

        @Override
        public void loop() {
            /*
             * Change the string "teamName" to your actual team name.
             */
        }
    }

}
