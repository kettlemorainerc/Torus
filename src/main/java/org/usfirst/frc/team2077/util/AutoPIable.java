package org.usfirst.frc.team2077.util;

import edu.wpi.first.wpilibj.Preferences;
import org.usfirst.frc.team2077.command.SparkMaxPIDTuner;

public abstract class AutoPIable {

    private String saveKey;

    public AutoPIable(String saveKey, double defaultP, double defaultI){
        this.saveKey = saveKey;
        Preferences.initDouble(String.format("%s_P", saveKey), defaultP);
        Preferences.initDouble(String.format("%s_I", saveKey), defaultI);

        setP(getSavedP());
        setI(getSavedI());
    }

    public AutoPIable(String saveKey){
        this(saveKey, 0.0, 0.0);
    }

    public abstract double getP();
    public abstract double getI();

    public abstract void setP(double p);
    public abstract void setI(double i);

    public double getSavedP(){
        return Preferences.getDouble(String.format("%s-P", saveKey), 0.0);
    }

    public double getSavedI(){
        return Preferences.getDouble(String.format("%s-I", saveKey), 0.0);
    }

    public void savePI(){
        Preferences.setDouble(String.format("%s_P", saveKey), getP());
        Preferences.setDouble(String.format("%s_I", saveKey), getI());
    }

    public abstract SparkMaxPIDTuner.ErrorMethod getErrorMethod();

}
