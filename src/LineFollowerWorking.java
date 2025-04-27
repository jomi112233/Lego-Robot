import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;



public class LineFollowerWorking{
    public static void main(String[] args){
        LightSensor lightData = new LightSensor();
        UltraSensor ultradata = new UltraSensor();
        
    
        Thread lightThread = new Thread(lightData);
        Thread ultraThread = new Thread(ultradata);

        lightThread.start();
        ultraThread.start();

        //wait until lightThread and ultraThread are ready
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
    private static final float KP = 300.0f;
    private static final float KI = 0.0f;
    private static final float KD = 75.0f;
    //motor tuning
    private static final int BASE_SPEED = 200;

    private static final int MAX_SPEED = 300;

    private static final float OBSTACLE_DISTANCE = 0.3f;
    
    private static float integral = 0;
    private static float lastError = 0;

    private boolean isAvoiding = false;
    private boolean obstacleAvoided = false;

    private int TrueSpeedA;
    private int TrueSpeedB;

    private int stepCounter = 0;
    //avoidance settings
    private int angleA = 0;
    private int driveDistance = 0;
    private int turnAngle = 0;
    private int avoidSpeed = 100;
    private int turnDirection = -1; //1 is left -1 is right
    private int straightAngle = 211; //to turn 90 degrees wheels need to turn this much
    //in mm
    private float wheelD = 0.056f;
    private float axelLenght = 0.077f;

    //lightSensor Object
    private LightSensor lightSensor;
    private UltraSensor ultraSensor;
    public calculator(LightSensor lightSensor, UltraSensor ultraSensor){
        this.lightSensor = lightSensor;
        this.ultraSensor = ultraSensor;
    }

    
    public void run(){
        
        //check if obstacle if no obstacle is detected use drive()
        //else start avoiding
        while (true) {
            float light = lightSensor.GetLight();
            float distance = ultraSensor.getDistance();
            
            if (distance <= OBSTACLE_DISTANCE && !isAvoiding) {
                obstacleAvoided = false;
                stepCounter = 0;
                Motor.A.resetTachoCount();
                isAvoiding = true;
            }
            
            if (isAvoiding) {
                Avoid(distance);
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

    private void Avoid(float distance) {
        switch (stepCounter) {
            //turns until the robot doesnt see an obstacle
            case 0:
                if(turnUntilObstacle(false, 1, distance)){

                    stepCounter = 1;
                    turnAngle = angleA;
                    float rotationRadians =(((float) angleA) / straightAngle * 90) / 180f * 3.14f;
                    driveDistance = Math.abs( (int) (OBSTACLE_DISTANCE * 360f / (Math.cos(rotationRadians) * Math.PI * wheelD)));
                    
                    LCD.drawString("DD: " + driveDistance + " TA: " + turnAngle, 0, 7);
                    if(driveDistance >= 1200){
                        driveDistance = 1200;
                    }

                    Motor.A.resetTachoCount();
                    LCD.drawString("case 0 ok: " + Motor.A.getTachoCount(), 0, 0);
                }
                
                break;
            //drives forward to calculated distanse
            case 1:
                if(driveUntilDistance(driveDistance)){
                    stepCounter = 2;
                    
                    Motor.A.resetTachoCount();
                    LCD.drawString("case 1 ok: " + Motor.A.getTachoCount(), 0, 1);
                }
                break;
            //turns back the distance turned in case 0
            case 2:
                if (turnUntilAngle(-turnAngle)) {
                    stepCounter = 3;
                    Motor.A.resetTachoCount();
                    LCD.drawString("case 2 ok: " + Motor.A.getTachoCount(), 0, 2);
                }
                break;
                //drives forward tiny bit
            case 3:
                if(driveUntilDistance(360)){
                    stepCounter = 4;
                    Motor.A.resetTachoCount();
                    LCD.drawString("case 3 ok: " + Motor.A.getTachoCount(), 0, 3);
                }
                break;
                //turns towards the obstacle
                //if obstacle is pressent turns back the same ammount and returns to case 2
                //else drives forward
            case 4:
                if(turnUntilObstacle(true, -1, distance)){
                    if(obstacleAvoided == false) {
                        turnAngle = angleA;
                        stepCounter = 2;
                        Motor.A.resetTachoCount();
                    } else {
                        stepCounter = 5;
                        Motor.A.resetTachoCount();
                        LCD.drawString("case 4 ok: " + Motor.A.getTachoCount(), 0, 4);
                    }
                }
                break;
                //drive forwards
            case 5: //checks furter to see if clear of the obstacle 70
                if(turnUntilAngle(70 * turnDirection)){
                    TrueSpeedA = avoidSpeed * 2;
                    TrueSpeedB = TrueSpeedA;
                    stepCounter = 6;
                    Motor.A.resetTachoCount();
                }
                break;

                //stops when the robot sees the line
            case 6:
                if(lightSensor.onLine() == true){
                    stepCounter = 7;
                    Motor.A.resetTachoCount();
                    LCD.drawString("case 5 ok: " + Motor.A.getTachoCount(), 0, 5);
                }
                break;

                //turns until robot is in line with the line and stops avoiding
            case 7:
                if(turnUntilAngle(straightAngle * turnDirection)){
                    isAvoiding = false;
                    Motor.A.resetTachoCount();
                    LCD.drawString("case 6 ok: " + Motor.A.getTachoCount(), 0, 6);
                }
                break;
            default:
                break;
        }
    }
    //searchForObstacle is true when we want to turn until we find obstacle
    //its false when we want to until we no longer see obstacle
    private boolean turnUntilObstacle(boolean searchForObstacle, int turnDirectionReverser, float dist){
        angleA = Motor.A.getTachoCount();
        TrueSpeedA = avoidSpeed * turnDirection * turnDirectionReverser;
        TrueSpeedB = -TrueSpeedA;

        if((dist < OBSTACLE_DISTANCE + 0.1) == searchForObstacle){
            return true;
        } else if(Math.abs(angleA) >= straightAngle + 70){
            obstacleAvoided = true;
            return true;
        } else {
            return false;
        }
        
    }
    //turns robot until given angle
    private boolean turnUntilAngle(int targetAngle){
        angleA = Motor.A.getTachoCount();
        int dynamicTurnDirection = -1;
        if(targetAngle > angleA){
            dynamicTurnDirection = 1;
        }
        TrueSpeedA = avoidSpeed * dynamicTurnDirection;
        TrueSpeedB = -TrueSpeedA;
        LCD.drawString("" + angleA + "" + Motor.A.getTachoCount(), 0, 0);
        if(Math.abs(angleA) + 1 >= Math.abs(targetAngle)){
            return true;
        } else {
            return false;
        }
    }
    //drives given distance
    private boolean driveUntilDistance(int targetAngle){
        angleA = Motor.A.getTachoCount();
        TrueSpeedA = avoidSpeed * 2;
        TrueSpeedB = TrueSpeedA;

        if(Math.abs(angleA) + 1 >= Math.abs(targetAngle)){
            return true;
        } else {
            return false;
        }
    }


    private void drive(float light){

        float targetValue = lightSensor.getTargetValue();
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
            LCD.drawString("" + Motor.A.getTachoCount(), 0, 1);
            LCD.drawString("" + Motor.A.getSpeed() + " " + Motor.A.getSpeed(), 0, 2);
            speedA = Calculator.GetSpeedA();
            speedB = Calculator.GetSpeedB();
        

            Motor.A.setSpeed(Math.abs(speedA));
            Motor.B.setSpeed(Math.abs(speedB));

            //drives forward or backwards with given  speed
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

//reads the ultra sonic sensor and returns its value
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

//reads the light sensor and returns its value
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

            if(lightSample[0] < targetValue){
                onLine = true;
            } else {
                onLine = false;
            }

            
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    //for calibrating the light sensor for better line following performance
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