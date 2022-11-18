#include "rn8209c_user.h"
#include "rn8209_flash.h"
#include "mk114_led.h"
#include "driver/uart.h"

#include "ArduinoJson.h"

static STU_8209C stu8209c_user;
extern rn8209c_flash stu8209c_flash;
extern "C" bool init_8209c_interface();
extern "C" void relay_open();

void RN8209CalibrateSuccessIndicate()
{
	stu8209c_flash.param = stu8209c_user;
    write_rn8209_param();
	LED_ALL_OFF;
	LED_BLUE_ON;
}

void RN8209CalibrateFailIndicate()
{
	LED_ALL_OFF;
	LED_RED_ON;
}

void RN8209CalibrateStartIndicate()
{
	LED_ALL_OFF;
	LED_PURPLE_ON;
}

void processRN8209(uint8_t cmd_mode)
{
	uint8_t uart_rxbuf[200];
	//uint8_t uart_txbuf[200];
	int buflen;
	static uint8_t mode = cmd_mode;
	static uint8_t calibrate_step = 0;
	static uint32_t  ref_voltage =0;
	static uint32_t  ref_current =0;
	static uint32_t  ref_power =0;
	static uint8_t  init_flag = 0;
	static uint8_t 	init_result =0;

//	DynamicJsonDocument doc(1024);
	DynamicJsonDocument resp(512);
/*
	if(cmd_mode)
	{
		buflen = uart_read_bytes(UART_NUM_0, uart_rxbuf, (uint32_t )200, 50/portTICK_PERIOD_MS);
		if (buflen)
		{
			DeserializationError error = deserializeJson(doc, uart_rxbuf);
			if (!error)
			{
				int inquire = doc["inquire"].as<int>() | 0;
				if (inquire > 0)
				{
					resp["inquire"] = inquire;
					if (inquire == 1)
					{
						resp["id"] = MAC;
						resp["type"] = DEV_NAME;
						resp["ver"] = FIRMWARE_VERSION;
						String output = resp.as<String>();
						uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
					}

					else if (inquire == 2)
					{
						resp["deviation"] = stu8209c_user.deviation;
						String output = resp.as<String>();
						uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
					}

					else if (inquire == 3)
					{
					 	//20210611 by ysh, read chip power after recv read cmd,to get current power info
						rn8209c_read_voltage(&rn8209_value.voltage);
						{
							uint8_t ret = rn8209c_read_emu_status();
							if(ret)
							{
								uint32_t temp_current=0 ;
								uint32_t temp_power=0 ;
								rn8209c_read_current(phase_A,&temp_current);
								rn8209c_read_power(phase_A,&temp_power);
								if(ret==1)
								{
									rn8209_value.current = temp_current;
									rn8209_value.power  = temp_power;
								}
								else
								{
									rn8209_value.current = (int32_t)temp_current*(-1);
									rn8209_value.power = (int32_t)temp_power *(-1);
								}
							}
						}
						resp["voltage"] = rn8209_value.voltage;
						resp["current"] = rn8209_value.current;
						resp["power"] = rn8209_value.power;
						String output = resp.as<String>();
						uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
					}
				}
				else
				{
					int set = doc["set"].as<int>() | 0;
					if (set > 0)
					{
						resp["set"] = set;

						if (set == 1)
						{
							if (!doc["power_start"].isNull() && !doc["EC"].isNull() &&
							    !doc["KV"].isNull() && !doc["R"].isNull()) {
								stu8209c_user.power_start = doc["power_start"].as<double>();
								stu8209c_user.EC = doc["EC"].as<int>();
								stu8209c_user.KV = doc["KV"].as<double>();
								stu8209c_user.R = doc["R"].as<int>();
								mode = 0;
								init_flag = 1;
								RN8209CalibrateStartIndicate();
								init_result = 0;
								resp["resp"] = "ack";
							} else {
								resp["resp"] = "nack";
							}

							String output = resp.as<String>();
							uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
						}
						else if (set == 2)
						{
							if (!doc["step"].isNull() && !doc["voltage"].isNull() &&
								!doc["current"].isNull() && !doc["power"].isNull() && init_result) {
								calibrate_step = doc["step"].as<int>();
								ref_voltage = doc["voltage"].as<int>();
								ref_current = doc["current"].as<int>();
								ref_power = doc["power"].as<int>();
								mode = 1;
								resp["resp"] = "ack";
							} else {
								resp["resp"] = "nack";
							}

							String output = resp.as<String>();
							uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
						}
						else if(set == 3)
						{
							int result = doc["result"].as<int>() | -1;
							int deviation = doc["deviation"].as<int>() | -1;

							if (result == 0 || result == 0) {
								resp["resp"] = "ack";
								stu8209c_user = read_stu8209c_calibrate_param();
								stu8209c_user.deviation = deviation;
								resp["GPQA"] = stu8209c_user.GPQA;
								resp["GPQB"] = stu8209c_user.GPQB;
								resp["PhsA"] = stu8209c_user.PhsA;
								resp["PhsB"] = stu8209c_user.PhsB;
								resp["Cst_QPhsCal"] = stu8209c_user.Cst_QPhsCal;
								resp["APOSA"] = stu8209c_user.APOSA;
								resp["APOSB"] = stu8209c_user.APOSB;
								resp["RPOSA"] = stu8209c_user.RPOSA;
								resp["RPOSB"] = stu8209c_user.RPOSB;
								resp["IARMSOS"] = stu8209c_user.IARMSOS;
								resp["IBRMSOS"] = stu8209c_user.IBRMSOS;
								resp["IBGain"] = stu8209c_user.IBGain;
								resp["Ku"] = stu8209c_user.Ku;
								resp["Kia"] = stu8209c_user.Kia;
								resp["Kib"] = stu8209c_user.Kib;
								if (result == 1) {
									RN8209CalibrateSuccessIndicate();
								} else {
									RN8209CalibrateFailIndicate();
								}
							}
							else
							{
								resp["resp"] = "nack";
							}

							String output = resp.as<String>();
							uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
						}
						else if(set == 4)
						{
							resp["resp"] = "ack";
							String output = resp.as<String>();
							uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
							// user_control();
						}
					}
				}
			}
		}
	}
*/
	switch(mode)
	{
		case 0: //��ʼ������
			/*if(init_8209c_interface()==1)
			{
				mode = 2;
				if(init_flag)
				{
					init_result = 1;
					init_flag = 0;
					resp["init_result"] = 1;
					String output = resp.as<String>();
					uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
				}
			}
			else
			{
				mode = 3;
				if(init_flag)
				{
					init_result = 1;
					init_flag = 0;
					resp["init_result"] = 0;
					String output = resp.as<String>();
					uart_write_bytes(UART_NUM_0, output.c_str(), output.length());
				}
			}
			break;*/
		case 1:  //У׼����
			if(calibrate_step==1)
			{
				rn8209c_calibrate_voltage_current(phase_A,ref_voltage,ref_current);
			}
			else if(calibrate_step==2)
			{
				rn8209c_calibrate_power_k_phase_a();
			}
			else if(calibrate_step==3)
			{
				rn8209c_calibrate_phs(phase_A,ref_power);
			}
			else if(calibrate_step==4)
			{
				rn8209c_calibrate_power_offset(phase_A,ref_power);
			}
			else if(calibrate_step==5)
			{
				rn8209c_calibrate_power_Q(phase_A,ref_power);
			}
			else if(calibrate_step==6)
			{
				rn8209c_calibrate_current_offset(phase_A);
			}
			rn8209c_read_voltage(&rn8209_value.voltage);
    		//	rn8209c_read_power(phase_A,&rn8209_value.power);
    		//	rn8209c_read_current(phase_A,&rn8209_value.current);
    			{
				uint8_t ret = rn8209c_read_emu_status();
				if(ret)
				{
					uint32_t temp_current=0 ;
					uint32_t temp_power=0 ;
					rn8209c_read_current(phase_A,&temp_current);
					rn8209c_read_power(phase_A,&temp_power);
					if(ret==1)
					{
						rn8209_value.current = temp_current;
						rn8209_value.power  = temp_power;
					}
					else
					{
						rn8209_value.current = (int32_t)temp_current*(-1);
						rn8209_value.power = (int32_t)temp_power *(-1);
					}
				}
			}
			{
				//resp["set"] = 2;
				//resp["step"] = calibrate_step;
				resp["voltage"] = rn8209_value.voltage;
				resp["current"] = rn8209_value.current;
				resp["power"] = rn8209_value.power;
				JsonObject data = resp.to<JsonObject>();
				pub("/RN8209toMQTT", data);
				//String output = resp.as<String>();
				//uart_write_bytes(UART_NUM_0, output.c_str(), output.length());

			}
			//mode = 2;
			break;
		case 2:
			rn8209c_read_voltage(&rn8209_value.voltage);
			{
				uint8_t ret = rn8209c_read_emu_status();
				if(ret)
				{
					uint32_t temp_current=0 ;
					uint32_t temp_power=0 ;
					rn8209c_read_current(phase_A,&temp_current);
					rn8209c_read_power(phase_A,&temp_power);
					if(ret==1)
					{
						rn8209_value.current = temp_current;
						rn8209_value.power  = temp_power;
					}
					else
					{
						rn8209_value.current = (int32_t)temp_current*(-1);
						rn8209_value.power = (int32_t)temp_power *(-1);
					}
				}
			}
    			//rn8209c_read_power(phase_A,&rn8209_value.power);
    			//rn8209c_read_current(phase_A,&rn8209_value.current);
			break;
		case 3:
			break;
	}
}
/*
void rn8209UartInit() {
    uart_config_t uart_config =
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 256, 256, 0, NULL,0);
}
*/
void rn8209_user_init(void *mode)
{
	/*if(*(uint8_t*)mode)
	{
		relay_open();
		//rn8209UartInit();
	}*/
	while(1)
	{
		processRN8209(1);
		delay(10000);
	}
}

void start_rn8209()
{
    //user_led_init();  // led init
	init_8209c_interface();
	//read_rn8209_param(); // remove for now, TODO: load calibration from flash
	xTaskCreate(rn8209_user_init, "rn8209c process", RN8209_TASK_STACK_SIZE, NULL, RN8209_TASK_PRIO, NULL);
}
