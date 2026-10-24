idf_component_register(
    SRCS "app_main.c" "task_nav.c" "task_audio.c" "task_vision.c"
         "drivers/vl53l0x.c" "drivers/mpu6050.c" "drivers/motor_tb6612.c"
         "drivers/encoder_pcnt.c" "drivers/servo.c"
    INCLUDE_DIRS "."
    REQUIRES aegis_core esp_driver_i2c esp_driver_i2s esp_driver_mcpwm esp_driver_pcnt
             esp_driver_ledc esp_driver_uart esp_driver_gpio esp_adc esp_timer)
