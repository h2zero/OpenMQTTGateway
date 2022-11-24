#ifdef ZsensorRN8209

#  include "ArduinoJson.h"
#  include "driver/uart.h"
#  include "mk114_led.h"
#  include "rn8209_flash.h"
#  include "rn8209c_user.h"

extern "C" bool init_8209c_interface();
extern "C" void relay_open();

StaticJsonDocument<512> doc;

void rn8209_loop(void* mode) {
  uint32_t voltage;
  int32_t current;
  int32_t power;

  while (1) {
    rn8209c_read_voltage(&voltage);
    uint8_t ret = rn8209c_read_emu_status();
    if (ret) {
      uint32_t temp_current = 0;
      uint32_t temp_power = 0;
      rn8209c_read_current(phase_A, &temp_current);
      rn8209c_read_power(phase_A, &temp_power);
      if (ret == 1) {
        current = temp_current;
        power = temp_power;
      } else {
        current = (int32_t)temp_current * (-1);
        power = (int32_t)temp_power * (-1);
      }
    }

    JsonObject data = doc.to<JsonObject>();
    data["voltage"] = (float)voltage / 1000.0;
    data["current"] = (float)current / 10000.0;
    data["power"] = (float)power / 10000.0;
    pub("/RN8209toMQTT", data);
    delay(10000);
  }
}

void start_rn8209() {
  //read_rn8209_param(); // remove for now, TODO: load calibration from flash
  STU_8209C cal = {0};
  cal.Ku = 18570;
  cal.Kia = 136702;
  cal.EC = 10000;
  set_user_param(cal);
  init_8209c_interface();
  xTaskCreate(rn8209_loop, "rn8209_loop", RN8209_TASK_STACK_SIZE, NULL, RN8209_TASK_PRIO, NULL);
}

#endif // ZsensorRN8209