idf_component_register(SRCS "cam_main.c" INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_http_server lwip esp_timer driver)
