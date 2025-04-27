import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;



public class LineFollower{
    public static void main(String[] args){
        LightSensor lightData = new LightSensor();
        UltraSensor ultradata = new UltraSensor();
        
    
        Thread lightThread = new Thread(lightData);
        Thread ultraThread = new Thread(ultradata);

        lightThread.start();
        ultraThread.start();

        while(!lightData.isReady() || !ultradata.isReady()){
            try {
                Thread.sleep(50);
            } catch(InterruptedException e) {

            }
        }

        calculator calculator = new calculator(lightData, ultradata);
        Thread calculatorThread = new Thread(calculator);
        Thread motorThread = new Thread(new Motors(calculator));
        
    
        calculatorThread.start();
        motorThread.start();
    }
}



class calculator implements Runnable{
    //drive PID settings
    private static final float KP = 250.0f;
    private static final float KI = 10.0f;
    private static final float KD = 150.0f;

    //avoid PID settings
    private static final float AKP = 800.0f;
    private static final float AKI = 30.0f;
    private static final float AKD = 400.0f;

    //motor tuning
    private static final int BASE_SPEED = 200;
    private static final int AVOID_BASE_SPEED = 200;

    private static final int MAX_SPEED = 300;
    private static final int AVOID_MAX_SPEED = 600;

    private static final float OBSTACLE_DISTANCE = 0.4f;
    
    private static float integral = 0;
    private static float lastError = 0;

    private boolean isAvoiding = false;

    private int TrueSpeedA;
    private int TrueSpeedB;

    //lightSensor Object
    private LightSensor lightSensor;
    private UltraSensor ultraSensor;
    public calculator(LightSensor lightSensor, UltraSensor ultraSensor){
        this.lightSensor = lightSensor;
        this.ultraSensor = ultraSensor;
    }


    public void run(){
        
        while (true) {
            float light = lightSensor.GetLight();
            float distance = ultraSensor.getDistance();
            LCD.drawString("Calc Light: " + light, 0, 1);
            if (distance <= OBSTACLE_DISTANCE && !isAvoiding) {
                isAvoiding = true;
            }
            
            if (isAvoiding) {
                Avoid();
            } else {
                drive(light);
            }
            
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void Avoid() {
        
        LCD.drawString("avoiding", 0, 2);

        float targetDistance = 0.3f;
        long escapeTime = 1500;
    
        float currentDistance = ultraSensor.getDistance();
    
        if (currentDistance > OBSTACLE_DISTANCE) {
            long escapeStart = System.currentTimeMillis();


            while (System.currentTimeMillis() - escapeStart < escapeTime) {

                // Keep moving sideways
                float error = -0.1f; // slight turn away
                float correction = AKP * error;
                int speedA = (int)(AVOID_BASE_SPEED + correction);
                int speedB = (int)(AVOID_BASE_SPEED - correction);
                
                speedA = Math.min(Math.max(speedA, -AVOID_MAX_SPEED), AVOID_MAX_SPEED);
                speedB = Math.min(Math.max(speedB, -AVOID_MAX_SPEED), AVOID_MAX_SPEED);

                TrueSpeedA = speedA;
                TrueSpeedB = speedB;

                try { Thread.sleep(10); } catch (InterruptedException e) {}
            }
    
            isAvoiding = false;
            LCD.clear();
            return;
        }
    
        float error = targetDistance - currentDistance; //avoid right
        //float error = currentDistance - targetDistance; //avoid left
    
        integral = 0.5f * integral + error;
        float derivative = error - lastError;
        lastError = error;
    
        float correction = AKP * error + AKI * integral + AKD * derivative;
        correction = Math.max(Math.min(correction, 300), -300);
    
        int speedA = (int)(AVOID_BASE_SPEED + correction);
        int speedB = (int)(AVOID_BASE_SPEED - correction);
    
        speedA = Math.min(Math.max(speedA, -AVOID_MAX_SPEED), AVOID_MAX_SPEED);
        speedB = Math.min(Math.max(speedB, -AVOID_MAX_SPEED), AVOID_MAX_SPEED);
    
        TrueSpeedA = speedA;
        TrueSpeedB = speedB;

        try { Thread.sleep(10); } catch (InterruptedException e) {}


    }
    
    
    private void drive(float light){
        //colorSample = colorSample[0]
        // float backgroundValue = 0.8f;
        // float lineValue = 0.2f;
        // float targetValue = (lineValue + backgroundValue) / 2;

        float targetValue = lightSensor.getTargetValue();
                    
        //colorProvider.fetchSample(colorSample, 0);
        float currentValue = light;
        
        //float error = targetValue - currentValue; // left side of track
        float error = currentValue - targetValue; // Right side of track
                        
        integral = integral * 0.5f + error;
        float derivative = error - lastError;
        float correction = KP * error + KI * integral + KD * derivative;
                    
        correction = Math.max(Math.min(correction, 300), -300);
                        
        int speedA = (int)(BASE_SPEED + correction);
        int speedB = (int)(BASE_SPEED - correction);
                        
        speedA = Math.min(Math.max(speedA, -MAX_SPEED), MAX_SPEED);
        speedB = Math.min(Math.max(speedB, -MAX_SPEED), MAX_SPEED);
                    
        TrueSpeedA = speedA;
        TrueSpeedB = speedB;
        
    }

    public int GetSpeedA(){
        return TrueSpeedA;
    }
    public int GetSpeedB(){
        return TrueSpeedB;
    }

}




class Motors implements Runnable {
    private calculator Calculator;
    public Motors(calculator Calculator){
        this.Calculator = Calculator;
    }

    public void run(){

        int speedA;
        int speedB;

        while (true) {

            speedA = Calculator.GetSpeedA();
            speedB = Calculator.GetSpeedB();
        

            Motor.A.setSpeed(Math.abs(speedA));
            Motor.B.setSpeed(Math.abs(speedB));

            
            if (speedA > 0) {
                Motor.A.forward();
            } else {
                Motor.A.backward();
            }
            
            if (speedB > 0) {
                Motor.B.forward();
            } else {
                Motor.B.backward();
            }

        }
    }
}

class UltraSensor implements Runnable {
    private float distance = Float.MAX_VALUE;
    private volatile boolean ready = false;

    

    public float getDistance(){
        return distance;
    }

    public boolean isReady(){
        return ready;
    }

    public void run(){
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] distanceSample = new float[distanceMode.sampleSize()];
        
        while (true) {
            distanceMode.fetchSample(distanceSample, 0);
            distance = distanceSample[0];
            ready = true;
        }
        
    }
}

class LightSensor implements Runnable {
    private float light = Float.MAX_VALUE;
    private volatile boolean ready = false;
    private volatile boolean onLine = false;

    private float lineValue = 0.0f;
    private float backgroundValue = 1.0f;
    private float targetValue = 0.5f; // midpoint between line and background

    public float GetLight() {
        return light;
    }

    public boolean isReady() {
        return ready;
    }

    public float getTargetValue() {
        return targetValue;
    }

    public boolean onLine(){
        return onLine;
    }

    public void run() {
        EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S2);
        SampleProvider lightProvider = lightSensor.getRedMode();
        float[] lightSample = new float[lightProvider.sampleSize()];

        calibrate(lightProvider, lightSample);  //start calibration

        while (true) {
            lightProvider.fetchSample(lightSample, 0);
            light = lightSample[0];
            ready = true;

            LCD.drawString("Light: " + light, 0, 0);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void calibrate(SampleProvider provider, float[] sample) {
        LCD.clear();
        LCD.drawString("Place on LINE", 0, 0);
        LCD.drawString("Press ENTER", 0, 1);

        while (!Button.ENTER.isDown()) {
            // Wait for ENTER
        }
        provider.fetchSample(sample, 0);
        lineValue = sample[0];
        LCD.clear();
        LCD.drawString("Line Value: " + lineValue, 0, 0);

        try { Thread.sleep(1000); } catch (InterruptedException e) {}

        LCD.clear();
        LCD.drawString("Place on BACKGROUND", 0, 0);
        LCD.drawString("Press ENTER", 0, 1);

        while (!Button.ENTER.isDown()) {
            // Wait for ENTER
        }
        provider.fetchSample(sample, 0);
        backgroundValue = sample[0];
        LCD.clear();
        LCD.drawString("Background: " + backgroundValue, 0, 0);

        // Calculate target midpoint
        targetValue = (lineValue + backgroundValue) / 2.0f;
        LCD.drawString("Target: " + targetValue, 0, 2);

        try { Thread.sleep(2000); } catch (InterruptedException e) {}
        LCD.clear();
    }
}