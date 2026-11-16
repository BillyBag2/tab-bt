idf_component_register(SRCS "main.cpp" "analyser.cpp" "ui.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash esp_timer esp_hosted m5stack_tab5)
