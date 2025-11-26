package org.firstinspires.ftc.teamcode;

public class FullyCombinedMecanumtraindriveSystem {

        // ---- CONFIGURABLE SETTINGS ----
        private double slowModeFactor = 0.35;     // Slow mode multiplier
        private double headingHoldKp = 4.5;       // Proportional correction for heading hold
        private double driftCompKp = 0.10;        // Drift autocorrection strength

        private boolean slowModeEnabled = false;
        private boolean fieldCentricEnabled = false;
        private boolean headingHoldEnabled = false;

        private double headingOffset = 0;         // Driver-selected heading zero
        private double targetHeading = 0;         // Heading-hold target

        // ---- MOTORS ----
        private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

        // ---- IMU ----
        private IMU imu;

        public MecanumDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, IMU imuDevice) {
            this.frontLeftMotor = fl;
            this.frontRightMotor = fr;
            this.backLeftMotor = bl;
            this.backRightMotor = br;
            this.imu = imuDevice;

            // Correct motor directions for real mecanum wheel geometry
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // Allow driver to toggle slow mode
        public void setSlowMode(boolean enabled) {
            slowModeEnabled = enabled;
        }

        // Allow driver to toggle field-centric mode
        public void setFieldCentric(boolean enabled) {
            fieldCentricEnabled = enabled;
        }

        // Allow driver to zero heading for field-centric
        public void zeroFieldHeading() {
            headingOffset = getRawHeading();
        }

        // Allow driver to enable heading hold
        public void enableHeadingHold(boolean enabled) {
            if (enabled && !headingHoldEnabled) {
                targetHeading = getHeading(); // Lock current angle
            }
            headingHoldEnabled = enabled;
        }

        // ---- Utility to get IMU heading ----
        private double getRawHeading() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        private double getHeading() {
            return getRawHeading() - headingOffset;
        }

        // ---- MAIN DRIVE METHOD ----
        public void drive(double throttle, double strafe, double spin) {

            // Slow mode
            if (slowModeEnabled) {
                throttle *= slowModeFactor;
                strafe   *= slowModeFactor;
                spin     *= slowModeFactor;
            }

            // RoadRunner-style input weighting
            double maxInput = Math.max(1.0, Math.abs(throttle) + Math.abs(strafe) + Math.abs(spin));
            throttle /= maxInput;
            strafe   /= maxInput;
            spin     /= maxInput;

            // Field-centric transform
            if (fieldCentricEnabled) {
                double heading = getHeading();
                double cosA = Math.cos(heading);
                double sinA = Math.sin(heading);

                double adjustedX = strafe * cosA - throttle * sinA;
                double adjustedY = strafe * sinA + throttle * cosA;

                strafe = adjustedX;
                throttle = adjustedY;
            }

            // Heading hold
            if (headingHoldEnabled) {
                double error = targetHeading - getHeading();
                spin += error * headingHoldKp;  // Add correction to spin input
            }

            // Advanced driver assist: drift correction during straight movement
            if (Math.abs(spin) < 0.05 && (Math.abs(throttle) > 0.1 || Math.abs(strafe) > 0.1)) {
                double drift = getHeading();
                spin -= drift * driftCompKp;
            }

            // ---- Mecanum power mixing ----
            double fl = throttle + strafe + spin;
            double fr = throttle - strafe - spin;
            double bl = throttle - strafe + spin;
            double br = throttle + strafe - spin;

            // Normalize wheel powers
            double largest = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))
            );

            if (largest > 1.0) {
                fl /= largest;
                fr /= largest;
                bl /= largest;
                br /= largest;
            }

            // ---- Set motor powers ----
            frontLeftMotor.setPower(fl);
            frontRightMotor.setPower(fr);
            backLeftMotor.setPower(bl);
            backRightMotor.setPower(br);
        }
    }



