
set(TOUCH_DIR "ft5x06")

idf_component_register(SRC_DIRS "." "${TOUCH_DIR}" "calibration" "calibration/basic_painter" "calibration/basic_painter/fonts"
                        INCLUDE_DIRS "." "${TOUCH_DIR}" "calibration/basic_painter" "calibration/basic_painter/fonts"
                        PRIV_INCLUDE_DIRS "calibration"
                        PRIV_REQUIRES bus nvs_flash
                        REQUIRES screen)
