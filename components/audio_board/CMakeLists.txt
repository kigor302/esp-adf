set(COMPONENT_ADD_INCLUDEDIRS ./include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES audio_sal audio_hal esp_dispatcher esp_peripherals display_service)


if (CONFIG_ESP_LYRAT_V4_2_BOARD)
message(STATUS "Current board name is " CONFIG_ESP_LYRAT_V4_2_BOARD)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./lyrat_v4_2)
set(COMPONENT_SRCS
./lyrat_v4_2/board.c
./lyrat_v4_2/board_pins_config.c
)
endif()

if (CONFIG_ESP_LYRAT_V4_3_BOARD)
message(STATUS "Current board name is " CONFIG_ESP_LYRAT_V4_3_BOARD)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./lyrat_v4_3)
set(COMPONENT_SRCS
            ./lyrat_v4_3/board.c
            ./lyrat_v4_3/board_pins_config.c
)
endif()

if (CONFIG_ESP_LYRATD_MINI_V1_1_BOARD)
message(STATUS "Current board name is " CONFIG_ESP_LYRATD_MINI_V1_1_BOARD)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./lyratd_mini_v1_1)
set(COMPONENT_SRCS
./lyratd_mini_v1_1/board.c
./lyratd_mini_v1_1/board_pins_config.c
)
endif()


if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD)
message(STATUS "Current board name is " CONFIG_ESP_LYRATD_MSC_V2_1_BOARD)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./lyratd_msc_v2_1)
set(COMPONENT_SRCS
./lyratd_msc_v2_1/board.c
./lyratd_msc_v2_1/board_pins_config.c
)
endif()

if (CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
message(STATUS "Current board name is " CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./lyratd_msc_v2_2)
set(COMPONENT_SRCS
./lyratd_msc_v2_2/board.c
./lyratd_msc_v2_2/board_pins_config.c
)
endif()

if (CONFIG_ESP_AITHINKER_V2_2_BOARD)
message(STATUS "Current board name is " CONFIG_ESP_AITHINKER_V2_2_BOARD)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./aithinker_v2_2)
set(COMPONENT_SRCS
            ./aithinker_v2_2/board.c
            ./aithinker_v2_2/board_pins_config.c
)
endif()

register_component()
