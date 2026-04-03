# List of source files in main component
set(srcs "main.c"
         "esp_hid_device_main.c"
         "esp_hid_gap.c")

# Include directories for headers
set(include_dirs ".")

# Register main component
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES esp_hid driver esp_adc nvs_flash
)
