package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RGBIndicatorLight {

    private Servo light;
    private Telemetry telemetry;

    private static final double MIN_PWM_US = 500.0;
    private static final double MAX_PWM_US = 2500.0;

    public void init(HardwareMap hwMap, Telemetry telemetry, String pwmPortName) {
        this.telemetry = telemetry;

        try {
            light = hwMap.get(Servo.class, pwmPortName);

            ((PwmControl) light).setPwmRange(
                    new PwmControl.PwmRange(MIN_PWM_US, MAX_PWM_US)
            );

            telemetry.addLine("RGB Indicator Light initialized.");
        } catch (Exception e) {
            telemetry.addData("⚠️ RGB Light not found", pwmPortName);
        }
    }

    // ----------------------
    // DIRECT COLOR POSITIONS
    // ----------------------

    public void red()     { setPos(0.10); }
    public void orange()  { setPos(0.20); }
    public void yellow()  { setPos(0.30); }
    public void green()   { setPos(0.40); }  // ← This will be true green
    public void cyan()    { setPos(0.50); }
    public void blue()    { setPos(0.60); }
    public void purple()  { setPos(0.70); }
    public void magenta() { setPos(0.80); }
    public void white()   { setPos(1.00); }

    private void setPos(double pos) {
        if (light != null) {
            ((PwmControl) light).setPwmEnable();     // enable PWM before setting color
            light.setPosition(pos);
            telemetry.addData("RGB Light", "Position: %.2f", pos);
        }
    }

    public void off() {
        if (light != null) {
            ((PwmControl) light).setPwmDisable();    // completely stop PWM → LED turns OFF
            telemetry.addLine("RGB Light OFF");
        }
    }


    public void stop() {
        off();
    }
}
