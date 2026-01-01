package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class RPM_per_dist {// =====================================================
    // DISTANCE → RPM FUNCTION
// Fill in constants once real data is collected
// =====================================================
    public double getFlywheelRPMForDistance(double distanceInches) {

        // ----------- FILL THESE IN LATER -----------
        double slopeRPMperInch = 25.0;   // RPM increase per inch
        double baseRPM         = 2850.0;   // RPM at zero distance
        double minRPM          = 2725.0;   // safety minimum
        double maxRPM          = 3000.0;   // safety maximum
        // -------------------------------------------

        // Basic linear model
        double rpm = (slopeRPMperInch * (distanceInches - 45) ) + baseRPM;

//        telemetry.addData("Flywheel rpm eq", rpm);

        // Clamp only if limits are defined
        if (maxRPM > minRPM && minRPM > 0) {
            rpm = Math.max(minRPM, Math.min(rpm, maxRPM));
        }

//        telemetry.addData("Flywheel rpm eq after", rpm);
        return rpm;
    }

}