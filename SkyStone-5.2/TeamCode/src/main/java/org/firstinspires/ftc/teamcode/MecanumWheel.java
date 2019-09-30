package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by peter on 2018/1/28.
 */

//@Disabled

public class MecanumWheel {
    float forward;
    float right;
    float clockwise;
    float fl_value;
    float fr_value;
    float rl_value;
    float rr_value;
    float max_value;
    public void init(){
        forward = 0;right = 0;clockwise = 0;
        fl_value = 0;fr_value = 0;rl_value = 0;rr_value = 0;
        max_value = 0;
    }
    public void setMode(float v_forward, float v_right, float v_clockwise){
        forward = - v_forward;
        right = - v_right;
        clockwise = v_clockwise;
    }
    public void calculatePower(){
        fl_value = (forward - clockwise + right);
        fr_value = (forward + clockwise - right);
        rl_value = (forward - clockwise - right);
        rr_value = (forward + clockwise + right);
        max_value = getMaxValue(fl_value,fr_value,rl_value,rr_value);
        if (max_value > 1.0 ) {
            fl_value = fl_value / max_value;
            fr_value = fr_value / max_value;
            rl_value = rl_value / max_value;
            rr_value = rr_value / max_value;
        }
    }

//    fl_value = (forward + clockwise + right);
//    fr_value = (forward - clockwise - right);
//    rl_value = (forward + clockwise - right);
//    rr_value = (forward - clockwise + right);
    private float getMaxValue(float para1, float para2, float para3, float para4){
        float f_max_value = (Math.abs(para1));
        if (Math.abs(para2) > f_max_value) f_max_value = Math.abs(para2);
        if (Math.abs(para3) > f_max_value) f_max_value = Math.abs(para3);
        if (Math.abs(para4) > f_max_value) f_max_value = Math.abs(para4);
        return f_max_value;
    }
}
