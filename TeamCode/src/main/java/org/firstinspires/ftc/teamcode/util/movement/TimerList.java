package org.firstinspires.ftc.teamcode.util.movement;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

/**
 * basically makes managing timers a ton easier
 */
public class TimerList {
    private final HashMap<String, ElapsedTime> timerList = new HashMap<>();

    //resets a timer. If it doesn't exist, make a new one
    public void resetTimer(String timerName){
        if(timerList.containsKey(timerName)){
            timerList.get(timerName).reset();
        } else {
            timerList.put(timerName, new ElapsedTime());
        }
    }
    public double getTimerMillis(String timerName){
        if(timerList.containsKey(timerName)){
            return timerList.get(timerName).milliseconds();
        } else {
            return 0;
        }
    }

    public double getTimerNano(String timerName){
        if(timerList.containsKey(timerName)){
            return timerList.get(timerName).nanoseconds();
        } else {
            return 0;
        }
    }
    public double getTimer(String timerName){
        if(timerList.containsKey(timerName)){
            return timerList.get(timerName).time();
        } else {
            return 0;
        }
    }

    public boolean checkTimePassed(String timerName, double milliseconds){
        if(timerList.containsKey(timerName)){
            return timerList.get(timerName).milliseconds() >= milliseconds;
        } else {
            return false; //if timer hasn't been started and this has been called, assume elapsed time is none seconds or somethign
        }
    }

    public boolean checkTimerExist(String timerName){
        return timerList.containsKey(timerName);
    }


}
