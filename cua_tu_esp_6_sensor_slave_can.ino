#include "cua_tu_esp_6_sensor_slave_can.h"


void getStatus(){
    ECHOLN("getStatus");
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = control_motor.device_id;
    tx_frame.FIR.B.DLC = 4;
    if(control_motor.forward){
        tx_frame.data.u8[0] = 0;        //1 - open;  0 - close
    }
    else{
        tx_frame.data.u8[0] = 1;        //1 - open;  0 - close
    }
    tx_frame.data.u8[1] = control_led.red_after;
    tx_frame.data.u8[2] = control_led.green_after;
    tx_frame.data.u8[3] = control_led.blue_after;
    ESP32Can.CANWriteFrame(&tx_frame);
}


void motor_init(){
    //set varialble
    control_motor.normal_mode = true;
    control_motor.forward = true;
    control_motor.daytay = true;
    control_motor.first_run = true;
    control_motor.pul_speed = 0;
    control_motor.pre_pul_speed = 0;
    control_motor.count_first_run = 0;
    control_motor.count_pul_FG = 0;

    control_motor.device_id = EEPROM.read(EEPROM_DEVICE_ID);
    ECHO("Device ID = ");
    ECHOLN(control_motor.device_id);

    if(EEPROM.read(EEPROM_DISTANT) != 255 && EEPROM.read(EEPROM_DISTANT) != 0){
        control_motor.is_save_distant = true;
        control_motor.count_pul_distant = EEPROM.read(EEPROM_DISTANT);
        ECHO("Distant = ");
        ECHOLN(control_motor.count_pul_distant);
    }else{
        control_motor.is_save_distant = false;
        ECHOLN("is_save_distant fasle!");
    }

    control_motor.mode_run = char(EEPROM.read(EEPROM_SET_MODE_RUN_BEGIN));
    if(control_motor.mode_run != 1 && control_motor.mode_run != 2 && control_motor.mode_run != 3 && control_motor.mode_run != 4 && control_motor.mode_run != 5){
        control_motor.mode_run = 3;
        ECHOLN("read set moderun EEPROM fail, auto set 3!");
    }else{
        ECHO("Mode Run =  ");
        ECHOLN(control_motor.mode_run);
    }

    if(EEPROM.read(EEPROM_SET_TIME_RETURN) != 255 && EEPROM.read(EEPROM_SET_TIME_RETURN) != 0){
        control_motor.time_return = EEPROM.read(EEPROM_SET_TIME_RETURN);
        ECHO("time_return = ");
        ECHO(control_motor.time_return);
        ECHOLN("0 (ms)");
    }else{
        control_motor.time_return = 35;
        ECHOLN("isSetTimeReurn fasle, auto set 350(ms)");
    }


    if(EEPROM.read(EEPROM_SET_PERCENT_OUT_LOW_SPEED) != 255 && EEPROM.read(EEPROM_SET_PERCENT_OUT_LOW_SPEED) != 0
        && EEPROM.read(EEPROM_SET_PERCENT_IN_LOW_SPEED) != 255 && EEPROM.read(EEPROM_SET_PERCENT_IN_LOW_SPEED) != 0){
        control_motor.percent_low_in = EEPROM.read(EEPROM_SET_PERCENT_IN_LOW_SPEED);
        ECHO("percentLowSpeedIn = ");
        ECHOLN(control_motor.percent_low_in);

        control_motor.percent_low_out = EEPROM.read(EEPROM_SET_PERCENT_OUT_LOW_SPEED);
        ECHO("percentLowSpeedOut = ");
        ECHOLN(control_motor.percent_low_out);
    }else{
        control_motor.percent_low_in = 20;
        control_motor.percent_low_out = 20;
        ECHOLN("isSavePercentLowSpeed fasle, auto set 20");
    }

    //count time check analog sensor
    control_motor.time_delay_analog = EEPROM.read(EEPROM_COUNT_TIME_ANALOG);
    if(control_motor.time_delay_analog == 0 || control_motor.time_delay_analog == 255){
        control_motor.time_delay_analog = 2;
    }
    ECHO("count_check_analog_pin = ");
    ECHO(control_motor.time_delay_analog*100);
    ECHOLN("ms");

    //value error analog 
    control_motor.define_error_analog = EEPROM.read(EEPROM_VALUE_ERROR_ANALOG);
    if(control_motor.define_error_analog == 0){
        control_motor.define_error_analog = 100;
    }
    ECHO("define Error Analog Read: ");
    ECHOLN(control_motor.define_error_analog);

    //time auto close
    control_motor.define_time_auto_close = EEPROM.read(EEPROM_TIME_AUTO_CLOSE);
    // if(definecontrol_motor.time_auto_close == 0){
    //     definecontrol_motor.time_auto_close = 10;       //10 min
    // }
    ECHO("Time Auto Close: ");
    ECHO(control_motor.define_time_auto_close);
    ECHOLN("min");


    control_motor.value_error_analog = analogRead(ANALOG_READ_BUTTON);
    control_motor.pre_value_error_analog = control_motor.value_error_analog;


}

void led_init(){
    //set varialble
    control_led.count_change_led = 0;

    control_led.red_after = char(EEPROM.read(EEPROM_WIFI_LED_RED));
	control_led.green_after  = char(EEPROM.read(EEPROM_WIFI_LED_GREEN));
    control_led.blue_after  = char(EEPROM.read(EEPROM_WIFI_LED_BLUE));
    control_led.red_before  = control_led.red_after;
    control_led.green_before  = control_led.green_after;
    control_led.blue_before  = control_led.blue_after;
	ledcWrite(LED_CHANNEL_R, control_led.red_after);
    ledcWrite(LED_CHANNEL_G, control_led.green_after);
    ledcWrite(LED_CHANNEL_B, control_led.blue_after);
    ECHO("red: ");
    ECHOLN(control_led.red_after);
    ECHO("green: ");
    ECHOLN(control_led.green_after);
    ECHO("blue: ");
    ECHOLN(control_led.blue_after);
}


void Open(){
    ECHOLN("open");

    control_motor.status_stop = false;
    control_motor.forward = true;
    control_motor.daytay = false;
    control_motor.count_calcu_speed = 0;
    control_led.status_led = true;
    digitalWrite(DIR, QUAY_THUAN);
    tickerCaculateSpeed.start();

	//bat den led
	if(control_led.count_change_led == 0){
		tickerSetPwmLedLightOff.stop();
		tickerSetPwmLedLightOn.start();
	}
}

void Close(){
    ECHOLN("close");

    control_motor.status_stop = false;
    control_motor.forward = false;
    control_motor.daytay = false;
    control_motor.count_calcu_speed = 0;
    
    digitalWrite(DIR, QUAY_NGHICH);
    tickerCaculateSpeed.start();
    tickerSetPwmLedLightOn.stop();
}


void Stop(){
    tickerCaculateSpeed.stop();

    if(control_motor.forward){
        digitalWrite(DIR, QUAY_NGHICH);     //cho dong co quay nghich
        ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_MAX);
        delay(control_motor.time_return*10);
        ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_STOP);
        control_motor.forward = false;
    }else{
        digitalWrite(DIR, QUAY_THUAN);
        ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_MAX);
        delay(control_motor.time_return*10);
        ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_STOP);
        control_motor.forward = true;
    }

    control_motor.status_stop = true;
    control_motor.daytay = true;
    control_motor.count_calcu_speed = 0;

    //reset_value_analog
    control_motor.value_error_analog = analogRead(ANALOG_READ_BUTTON);
    control_motor.pre_value_error_analog = control_motor.value_error_analog;

    getStatus();


}


void caculateSpeed(){
    //ECHOLN("speed");
    if(control_motor.count_calcu_speed <= 30){
        control_motor.count_calcu_speed++;
    }
    control_motor.speed_velectory = (control_motor.pul_speed - control_motor.pre_pul_speed)/(0.1*6);
    control_motor.pre_pul_speed = control_motor.pul_speed;
    
    // ECHO("van toc: ");
    // ECHOLN(speed);
    if(abs(control_motor.speed_velectory) <= MINSPEED && control_motor.count_calcu_speed >= 5){   //sau 5 lan chay thi moi tinh den van toc
        ECHOLN("Da dung lai");
        tickerCaculateSpeed.stop();

        if(control_motor.is_save_distant && control_motor.first_run){
            control_motor.first_run = false;
            control_motor.count_pul_FG = 0;
            control_motor.pre_pul_speed = 0;
        }
        

        if(control_motor.first_run && control_motor.count_first_run < 5){
            control_motor.count_first_run++;
        }
        
        if(control_motor.first_run == true && control_motor.count_first_run == 1){
            control_motor.count_pul_FG = 0;
            control_motor.pre_pul_speed = 0;
        }

        else if(control_motor.first_run == true && control_motor.count_first_run == 2){
            control_motor.count_pul_distant = abs(control_motor.count_pul_FG);
            EEPROM.write(EEPROM_DISTANT, control_motor.count_pul_distant);
            EEPROM.commit();
            control_motor.is_save_distant = true;
            if(control_motor.count_pul_FG < 0){
                control_motor.count_pul_FG = 0;
                control_motor.pre_pul_speed = 0;
            }
            control_motor.first_run = false;
        }


        if(!control_motor.first_run && control_motor.count_pul_FG <= 3){
			//tat den
			tickerSetPwmLedLightOff.stop();
			tickerSetPwmLedLightOn.stop();
			control_led.count_change_led = 0;
			ledcWrite(LED_CHANNEL_R, 0);
            ledcWrite(LED_CHANNEL_G, 0);
            ledcWrite(LED_CHANNEL_B, 0);

			control_motor.count_pul_FG = 0;
        }else if(!control_motor.first_run && (control_motor.count_pul_distant -3) <= control_motor.count_pul_FG){
            //bat den
			tickerSetPwmLedLightOff.stop();
			tickerSetPwmLedLightOn.stop();
			control_led.count_change_led = 0;
			ledcWrite(LED_CHANNEL_R, uint8_t(control_led.red_after));
            ledcWrite(LED_CHANNEL_G, uint8_t(control_led.green_after));
            ledcWrite(LED_CHANNEL_B, uint8_t(control_led.blue_after));
			
			control_motor.count_pul_FG = control_motor.count_pul_distant;
        }
        
        control_motor.count_calcu_speed = 0;

        Stop();
    }
}



void setLedApMode() {
    digitalWrite(ledTestWifi, !digitalRead(ledTestWifi));
}

void setPwmLedLighton(){
    control_led.count_change_led++;
    float out_led_red, out_led_green, out_led_blue;
    out_led_red = (float)0 + (((float)control_led.red_after - (float)0)/255)*control_led.count_change_led;
    out_led_red = abs(out_led_red);
    ledcWrite(LED_CHANNEL_R, uint8_t(out_led_red));

    out_led_green = (float)0 + (((float)control_led.green_after - (float)0)/255)*control_led.count_change_led;
    out_led_green = abs(out_led_green);
    ledcWrite(LED_CHANNEL_G, uint8_t(out_led_green));

    out_led_blue = (float)0 + (((float)control_led.blue_after - (float)0)/255)*control_led.count_change_led;
    out_led_blue = abs(out_led_blue);
    ledcWrite(LED_CHANNEL_B, uint8_t(out_led_blue));

    if(control_led.count_change_led == 255){
        ECHOLN("On Led");
		control_led.count_change_led = 0;
	}

}

void setPwmLedLightoff(){
    control_led.count_change_led++;
	float out_led_red, out_led_green, out_led_blue;
    out_led_red = (float)control_led.red_before + (((float)0 - (float)control_led.red_before)/255)*control_led.count_change_led;
    out_led_red = abs(out_led_red);
	ledcWrite(LED_CHANNEL_R, uint8_t(out_led_red));
    
    out_led_green = (float)control_led.green_before + (((float)0 - (float)control_led.green_before)/255)*control_led.count_change_led;
    out_led_green = abs(out_led_green);
	ledcWrite(LED_CHANNEL_G, uint8_t(out_led_green));
    
    out_led_blue = (float)control_led.blue_before + (((float)0 - (float)control_led.blue_before)/255)*control_led.count_change_led;
    out_led_blue = abs(out_led_blue);
	ledcWrite(LED_CHANNEL_B, uint8_t(out_led_blue));
    
    if(control_led.count_change_led == 255){
        ECHOLN("Off Led");
		control_led.count_change_led = 0;
	}

}

void setPwmLedLightChange(){
    control_led.count_change_led++;
    float out_led_red, out_led_green, out_led_blue;
    out_led_red = (float)control_led.red_before + (((float)control_led.red_after - (float)control_led.red_before)/255)*control_led.count_change_led;
    out_led_red = abs(out_led_red);
    ledcWrite(LED_CHANNEL_R, uint8_t(out_led_red));

    out_led_green = (float)control_led.green_before + (((float)control_led.green_after - (float)control_led.green_before)/255)*control_led.count_change_led;
    out_led_green = abs(out_led_green);
    ledcWrite(LED_CHANNEL_G, uint8_t(out_led_green));

    out_led_blue = (float)control_led.blue_before + (((float)control_led.blue_after - (float)control_led.blue_before)/255)*control_led.count_change_led;
    out_led_blue = abs(out_led_blue);
    ledcWrite(LED_CHANNEL_B, uint8_t(out_led_blue));

    // ECHO(uint8_t(out_led_red));
    // ECHO("-----");
    // ECHO(uint8_t(out_led_green));
    // ECHO("-----");
    // ECHOLN(uint8_t(out_led_blue));
    if(control_led.count_change_led == 255){
        ECHOLN("Change Led");
		control_led.count_change_led = 0;
        control_led.red_before = control_led.red_after;
        control_led.green_before = control_led.green_after;
        control_led.blue_before = control_led.blue_after;
	}
}



void IRAM_ATTR dirhallSensor1(){      //nhan du lieu tu cam bien ben ngoai
    if(control_motor.sensor_hall_duplicate != 1){
        control_motor.sensor_hall_duplicate = 1;
        ECHOLN("1");
        if(control_motor.forward){
            control_motor.count_pul_FG++;
        }else{
            control_motor.count_pul_FG--;
        }

        if(control_motor.daytay && control_motor.status_stop){
            // control_motor.sensor_hall_duplicate = 0;
            if(control_motor.hall_sensor_2 == true){
                //cho dong co chay thuan
                control_motor.hall_sensor_2 = false;
                // ECHOLN("open");
                Open();
            }
            else if(control_motor.hall_sensor_6 == true){
                //cho dong co chay nghich
                control_motor.hall_sensor_6 = false;
                // ECHOLN("close");
                Close();
            }
            else{
                control_motor.hall_sensor_1 = true;
            }
        }
    }
    
}

void IRAM_ATTR dirhallSensor2(){      //nhan du lieu tu cam bien ben ngoai
    if(control_motor.sensor_hall_duplicate != 2){
        control_motor.sensor_hall_duplicate = 2;
        ECHOLN("2");
        if(control_motor.forward){
            control_motor.count_pul_FG++;
        }else{
            control_motor.count_pul_FG--;
        }

        if(control_motor.daytay && control_motor.status_stop){
            // control_motor.sensor_hall_duplicate = 0;
            if(control_motor.hall_sensor_3 == true){
                //cho dong co chay thuan
                control_motor.hall_sensor_3 = false;
                // ECHOLN("open");
                Open();
            }
            else if(control_motor.hall_sensor_1 == true){
                //cho dong co chay nghich
                control_motor.hall_sensor_1 = false;
                // ECHOLN("close");
                Close();
            }
            else{
                control_motor.hall_sensor_2 = true;
            }
        }
    }
    
}

void IRAM_ATTR dirhallSensor3(){      //nhan du lieu tu cam bien ben ngoai
    if(control_motor.sensor_hall_duplicate != 3){
        control_motor.sensor_hall_duplicate = 3;
        ECHOLN("3");
        if(control_motor.forward){
            control_motor.count_pul_FG++;
        }else{
            control_motor.count_pul_FG--;
        }

        if(control_motor.daytay && control_motor.status_stop){
            // control_motor.sensor_hall_duplicate = 0;
            if(control_motor.hall_sensor_4 == true){
                //cho dong co chay thuan
                control_motor.hall_sensor_4 = false;
                // ECHOLN("open");
                Open();
            }
            else if(control_motor.hall_sensor_2 == true){
                //cho dong co chay nghich
                control_motor.hall_sensor_2 = false;
                // ECHOLN("close");
                Close();
            }
            else{
                control_motor.hall_sensor_3 = true;
            }
        }
    }
    
}

void IRAM_ATTR dirhallSensor4(){      //nhan du lieu tu cam bien ben ngoai
    if(control_motor.sensor_hall_duplicate != 4){
        control_motor.sensor_hall_duplicate = 4;
        ECHOLN("4");
        if(control_motor.forward){
            control_motor.count_pul_FG++;
        }else{
            control_motor.count_pul_FG--;
        }

        if(control_motor.daytay && control_motor.status_stop){
            // control_motor.sensor_hall_duplicate = 0;
            if(control_motor.hall_sensor_5 == true){
                //cho dong co chay thuan
                control_motor.hall_sensor_5 = false;
                // ECHOLN("open");
                Open();
            }
            else if(control_motor.hall_sensor_3 == true){
                //cho dong co chay nghich
                control_motor.hall_sensor_3 = false;
                // ECHOLN("close");
                Close();
            }
            else{
                control_motor.hall_sensor_4 = true;
            }
        }
    }
    
}

void IRAM_ATTR dirhallSensor5(){      //nhan du lieu tu cam bien ben ngoai
    if(control_motor.sensor_hall_duplicate != 5){
        control_motor.sensor_hall_duplicate = 5;
        ECHOLN("5");
        if(control_motor.forward){
            control_motor.count_pul_FG++;
        }else{
            control_motor.count_pul_FG--;
        }

        if(control_motor.daytay && control_motor.status_stop){
            // control_motor.sensor_hall_duplicate = 0;
            if(control_motor.hall_sensor_6 == true){
                //cho dong co chay thuan
                control_motor.hall_sensor_6 = false;
                // ECHOLN("open");
                Open();
            }
            else if(control_motor.hall_sensor_4 == true){
                //cho dong co chay nghich
                control_motor.hall_sensor_4 = false;
                // ECHOLN("close");
                Close();
            }
            else{
                control_motor.hall_sensor_5 = true;
            }
        }
    }
    
}

void IRAM_ATTR dirhallSensor6(){      //nhan du lieu tu cam bien ben ngoai
    if(control_motor.sensor_hall_duplicate != 6){
        control_motor.sensor_hall_duplicate = 6;
        ECHOLN("6");
        if(control_motor.forward){
            control_motor.count_pul_FG++;
        }else{
            control_motor.count_pul_FG--;
        }

        if(control_motor.daytay && control_motor.status_stop){
            // control_motor.sensor_hall_duplicate = 0;
            if(control_motor.hall_sensor_1 == true){
                //cho dong co chay thuan
                control_motor.hall_sensor_1 = false;
                // ECHOLN("open");
                Open();
            }
            else if(control_motor.hall_sensor_5 == true){
                //cho dong co chay nghich
                control_motor.hall_sensor_5 = false;
                // ECHOLN("close");
                Close();
            }
            else{
                control_motor.hall_sensor_6 = true;
            }
        }
    }
    
}


void IRAM_ATTR inputSpeed(){
    if(control_motor.forward){
        control_motor.pul_speed++;
    }else{
        control_motor.pul_speed--;
    }
    // ECHOLN(control_motor.pul_speed);
}



void tickerupdate(){
    tickerCaculateSpeed.update();
    tickerSetApMode.update();
	tickerSetPwmLedLightOn.update();
	tickerSetPwmLedLightOff.update();
    tickerSetPwmLedLightChange.update();
}


void checkAutoClose(){
    //reset lai bien control_motor.flag_auto_close
    if(control_motor.forward && control_motor.status_stop && control_motor.flag_auto_close){
        control_motor.flag_auto_close = false;
    }
    //setup bien control_motor.flag_auto_close len true, day la thoi diem tu mo va bat dau tinh thoi gian
    if(!control_motor.forward && control_motor.status_stop  && control_motor.count_pul_FG >= (control_motor.count_pul_distant - 3) && !control_motor.flag_auto_close){
        control_motor.time_auto_close = millis();
        control_motor.flag_auto_close = true;
        ECHO("Start time auto close: ");
        ECHO(control_motor.define_time_auto_close);
        ECHOLN("(min)");
    }
    if(control_motor.flag_auto_close && millis() >= control_motor.time_auto_close + control_motor.define_time_auto_close*1000*60){       //don vi tinh theo phut
        control_motor.flag_auto_close = false;
        Close();
    }
}


void setSpeedControl(){
    float inside = ((float)control_motor.percent_low_in/100)*control_motor.count_pul_distant;
    float outside = ((100 - (float)control_motor.percent_low_out)/100)*control_motor.count_pul_distant;
    if(!control_motor.first_run && !control_motor.status_stop && control_motor.forward && control_motor.count_pul_FG < outside){
        ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_MAX);
        // ECHOLN("MOTOR_MODE_SPEED_MAX");
    }
    
    else if(!control_motor.first_run && !control_motor.status_stop && !control_motor.forward && control_motor.count_pul_FG > inside){
        ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_MAX);  
        // ECHOLN("MOTOR_MODE_SPEED_MAX"); 
    }
    
    else if(!control_motor.status_stop && (control_motor.first_run || (control_motor.forward && control_motor.count_pul_FG > outside) || (!control_motor.forward && control_motor.count_pul_FG < inside))){
        switch (control_motor.mode_run)
        {
        case 1:
            ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_1);
            // ECHOLN("MOTOR_MODE_SPEED_1");
            break;
        case 2:
            ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_2);
            // ECHOLN("MOTOR_MODE_SPEED_2");
            break;
        case 3:
            ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_3);
            // ECHOLN("MOTOR_MODE_SPEED_3");
            break;
        case 4:
            ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_4);
            // ECHOLN("MOTOR_MODE_SPEED_4");
            break;
        default:
            break;
        }
    }
    // else{
    //     ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_STOP);
    // }
}

void checkAnalogReadButton(){
    //analogRead
    if(control_motor.status_stop && control_motor.forward && control_motor.count_pul_FG <= 3 && abs(millis() - control_motor.time_check_analog) > TIME_CHECK_ANALOG){
        control_motor.time_check_analog = millis();
        control_motor.value_error_analog = analogRead(ANALOG_READ_BUTTON);
        if(abs(control_motor.value_error_analog - control_motor.pre_value_error_analog) > control_motor.define_error_analog){
            for(int i = 0; i < control_motor.time_delay_analog; i++){
                control_motor.value_error_analog = analogRead(ANALOG_READ_BUTTON);
                if(abs(control_motor.value_error_analog - control_motor.pre_value_error_analog) <= control_motor.define_error_analog){
                    return;
                }
                delay(100);
            }
            ECHOLN("Analog Sensor!");
            Open();
        }
        
    }

}

void receiveDataCan(){
    CAN_frame_t rx_frame;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

        if (rx_frame.FIR.B.FF == CAN_frame_std) {
        printf("New standard frame");
        }
        else {
        printf("New extended frame");
        }

        if (rx_frame.FIR.B.RTR == CAN_RTR) {
        printf(" RTR from %d, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
        }
        else {
        printf(" from %d, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
            printf("%d ", rx_frame.data.u8[i]);
        }
        printf("\n");
        }

        if(rx_frame.MsgID == MSG_MASTER_ID){
            if(rx_frame.data.u8[0] == MSG_SET_ID){
                control_motor.device_id = rx_frame.data.u8[1];
                ECHO("Writed: ");
                ECHO(control_motor.device_id);
                EEPROM.write(EEPROM_DEVICE_ID, char(control_motor.device_id));
                EEPROM.commit();
            }
            if(rx_frame.data.u8[0] == control_motor.device_id){
                switch (rx_frame.data.u8[1])
                {
                    case MSG_GET_STATUS:
                        getStatus();
                        break;
                    case MSG_CONTROL_OPEN:
                        if (control_motor.count_pul_FG <= 3)
                        {
                            Open();
                        }
                        break;
                    case MSG_CONTROL_CLOSE:
                        if(control_motor.count_pul_FG >= (control_motor.count_pul_distant - 3)){
                            Close();
                        }
                        break;
                    case MSG_CONTROL_STOP:
                        Stop();
                        break;
                    case MSG_CONTROL_LED_VOICE:
                        if(control_led.status_led){
                            control_led.red_after = rx_frame.data.u8[2];
                            control_led.green_after = rx_frame.data.u8[3];
                            control_led.blue_after = rx_frame.data.u8[4];
                            EEPROM.write(EEPROM_WIFI_LED_RED, char(control_led.red_after));
                            EEPROM.write(EEPROM_WIFI_LED_GREEN, char(control_led.green_after));
                            EEPROM.write(EEPROM_WIFI_LED_BLUE, char(control_led.blue_after));
                            EEPROM.commit();
                            tickerSetPwmLedLightChange.start();
                        }
                        break;
                    case MSG_CONTROL_LED_HAND:
                        if(control_led.status_led){
                            control_led.red_after = rx_frame.data.u8[2];
                            control_led.green_after = rx_frame.data.u8[3];
                            control_led.blue_after = rx_frame.data.u8[4];
                            ledcWrite(LED_CHANNEL_R, control_led.red_after);
                            ledcWrite(LED_CHANNEL_G, control_led.green_after);
                            ledcWrite(LED_CHANNEL_B, control_led.blue_after);
                            control_led.red_before  = control_led.red_after;
                            control_led.green_before  = control_led.green_after;
                            control_led.blue_before  = control_led.blue_after;
                        }
                        break;
                    case MSG_RESET_DISTANT:
                        EEPROM.write(EEPROM_DISTANT, 0);
                        EEPROM.commit();
                        control_motor.is_save_distant = false;
                        control_motor.first_run = true;
                        control_motor.count_first_run = 0;
                        ECHOLN("resetDistant");
                        break;
                    case MSG_TIME_RETURN:
                        control_motor.time_return = rx_frame.data.u8[2];
                        ECHO("Writed: ");
                        ECHO(control_motor.time_return);
                        EEPROM.write(EEPROM_SET_TIME_RETURN, char(control_motor.time_return));
                        EEPROM.commit();
                        break;
                    case MSG_MODE_RUN:
                        control_motor.mode_run = rx_frame.data.u8[2];
                        ECHO("Writed: ");
                        ECHO(control_motor.mode_run);
                        EEPROM.write(EEPROM_SET_MODE_RUN_BEGIN, char(control_motor.mode_run));
                        EEPROM.commit();
                        break;
                    case MSG_PERCENT_LOW:
                        control_motor.percent_low_in = rx_frame.data.u8[2];
                        control_motor.percent_low_out = rx_frame.data.u8[3];
                        ECHO("Writed: ");
                        ECHO(control_motor.percent_low_out);
                        ECHO(",");
                        ECHOLN(control_motor.percent_low_in);
                        EEPROM.write(EEPROM_SET_PERCENT_OUT_LOW_SPEED, char(control_motor.percent_low_out));
                        EEPROM.write(EEPROM_SET_PERCENT_IN_LOW_SPEED, char(control_motor.percent_low_in));
                        EEPROM.commit();
                        break;
                    case MSG_DELAY_ANALOG:
                        control_motor.time_delay_analog = rx_frame.data.u8[2];
                        ECHO("Writed: ");
                        ECHO(control_motor.time_delay_analog);
                        EEPROM.write(EEPROM_COUNT_TIME_ANALOG, char(control_motor.time_delay_analog));
                        EEPROM.commit();
                        break;
                    case MSG_ERROR_ANALOG:
                        control_motor.define_error_analog = rx_frame.data.u8[2];
                        ECHO("Writed: ");
                        ECHO(control_motor.define_error_analog);
                        EEPROM.write(EEPROM_VALUE_ERROR_ANALOG, char(control_motor.define_error_analog));
                        EEPROM.commit();
                        break;
                    case MSG_AUTO_CLOSE:
                        control_motor.define_time_auto_close = rx_frame.data.u8[2];
                        ECHO("Writed: ");
                        ECHO(control_motor.define_time_auto_close);
                        EEPROM.write(EEPROM_TIME_AUTO_CLOSE, char(control_motor.define_time_auto_close));
                        EEPROM.commit();
                        break;
                    default:
                        break;
                }
                
            }
            
        }
    }
}

String MacID(){
    uint8_t mac[WL_MAC_ADDR_LENGTH];
    String macID;
    WiFi.mode(WIFI_AP);
    WiFi.softAPmacAddress(mac);
    macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) + String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
    macID.toUpperCase();
    return macID;
}

void SetupConfigMode(){
    ECHOLN("[WifiService][setupAP] Open AP....");
    WiFi.softAPdisconnect();
    WiFi.disconnect();
    server.close();
    delay(500);
    WiFi.mode(WIFI_AP_STA);
    String SSID_AP_MODE = SSID_PRE_AP_MODE + MacID();
    WiFi.softAP(SSID_AP_MODE.c_str(), PASSWORD_AP_MODE);
    IPAddress APIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(APIP, gateway, subnet);
    ECHOLN(SSID_AP_MODE);

    ECHOLN("[WifiService][setupAP] Softap is running!");
    IPAddress myIP = WiFi.softAPIP();
    ECHO("[WifiService][setupAP] IP address: ");
    ECHOLN(myIP);
}


void StartConfigServer(){    
    ECHOLN("[HttpServerH][startConfigServer] Begin create new server...");
    server.on("/config", HTTP_POST, ConfigMode);


    /*return index page which is stored in serverIndex */
    server.on("/", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", loginIndex);
    });
    server.on("/serverIndex", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", serverIndex);
    });
    /*handling uploading firmware file */
    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        ESP.restart();
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { //true to set the size to the current progress
                Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
        }
    });


    server.begin();
    ECHOLN("[HttpServerH][startConfigServer] HTTP server started");
}

void ConfigMode(){
    StaticJsonBuffer<RESPONSE_LENGTH> jsonBuffer;
    ECHOLN(server.arg("plain"));
    JsonObject& rootData = jsonBuffer.parseObject(server.arg("plain"));
    ECHOLN("--------------");
    if (rootData.success()) {
        server.sendHeader("Access-Control-Allow-Headers", "*");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json; charset=utf-8", "{\"status\":\"success\"}");
        
        String nid = rootData["deviceid"];

        control_motor.device_id = nid.toInt();

        ECHOLN("");
        ECHOLN("writing eeprom device id:"); 
        ECHO("Wrote: ");
        EEPROM.write(EEPROM_DEVICE_ID, control_motor.device_id);
        ECHOLN(control_motor.device_id);


        EEPROM.commit();
        ECHOLN("Done writing!");

        control_motor.normal_mode = true;
        tickerSetApMode.stop();
        digitalWrite(ledTestWifi, HIGH);

    }
    ECHOLN("Wrong data!!!");
}


void checkButtonConfigClick(){
    //hold to config mode
    if(digitalRead(PIN_CONFIG) == HIGH){
        configAPmode.time_click_button_config = millis();
    }
    if(digitalRead(PIN_CONFIG) == LOW && (configAPmode.time_click_button_config + CONFIG_HOLD_TIME) <= millis()){
        configAPmode.time_click_button_config = millis();
        tickerSetApMode.start();
        control_motor.normal_mode = false;
        SetupConfigMode();
        StartConfigServer();
    }
}

void setup() {
  // put your setup code here, to run once:
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    Serial.begin(115200);
    EEPROM.begin(EEPROM_WIFI_MAX_CLEAR);

     
    ledcSetup(LED_CHANNEL_R, 1000, 8); // 1 kHz PWM, 8-bit resolution
    ledcSetup(LED_CHANNEL_G, 1000, 8); // 1 kHz PWM, 8-bit resolution
    ledcSetup(LED_CHANNEL_B, 1000, 8); // 1 kHz PWM, 8-bit resolution
    ledcSetup(MOTOR_CHANNEL, 30000, 8); // 30 kHz PWM, 8-bit resolution

    ledcAttachPin(PIN_LED_LIGHT_R, LED_CHANNEL_R); // analog pin to channel led_R
    ledcAttachPin(PIN_LED_LIGHT_G, LED_CHANNEL_G); // analog pin to channel led_G
    ledcAttachPin(PIN_LED_LIGHT_B, LED_CHANNEL_B); // analog pin to channel led_B
    ledcAttachPin(PWM, MOTOR_CHANNEL);              // analog pin to channel Motor
    
    ECHOLN("");
    ECHOLN("START!!!");
    pinMode(DIR, OUTPUT);
    pinMode(PIN_CONFIG, INPUT_PULLUP);
    pinMode(inputFG, INPUT_PULLUP);
    pinMode(hallSensor1, INPUT_PULLUP);
    pinMode(hallSensor2, INPUT_PULLUP);
    pinMode(hallSensor3, INPUT_PULLUP);
    pinMode(hallSensor4, INPUT_PULLUP);
    pinMode(hallSensor5, INPUT_PULLUP);
    pinMode(hallSensor6, INPUT_PULLUP);
    pinMode(ledTestWifi, OUTPUT);
    digitalWrite(ledTestWifi, HIGH);
    
    motor_init();
    led_init();


    attachInterrupt(digitalPinToInterrupt(hallSensor1), dirhallSensor1, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor2), dirhallSensor2, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor3), dirhallSensor3, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor4), dirhallSensor4, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor5), dirhallSensor5, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor6), dirhallSensor6, RISING);
    attachInterrupt(digitalPinToInterrupt(inputFG), inputSpeed, FALLING);

    CAN_cfg.speed = CAN_SPEED_125KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_25;
    CAN_cfg.rx_pin_id = GPIO_NUM_33;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();

    Close();

    // ledcWrite(MOTOR_CHANNEL, MOTOR_MODE_SPEED_3);
}

void loop() {
    // put your main code here, to run repeatedly:

    if(!control_motor.forward && control_led.status_led && !control_motor.status_stop  && control_motor.count_pul_FG <= 10){
        control_led.status_led = false;
        tickerSetPwmLedLightOn.stop();
        tickerSetPwmLedLightOff.start();
    }
    if(control_motor.define_time_auto_close != 0){
        checkAutoClose();
    }
    checkAnalogReadButton();
    checkButtonConfigClick();
    setSpeedControl();
    receiveDataCan();
    tickerupdate();
    if(!control_motor.normal_mode){
        server.handleClient();
    }

}
