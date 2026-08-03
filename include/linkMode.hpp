#pragma once

// Serial1 carries ONE of these, never both: the two ESP-CAM boards run at
// different bauds and send in opposite directions. Flip ESP_LINK_MODE and
// reflash - baud, telemetry and which task starts all follow from it.
#define ESP_LINK_SD     0 // sd_logging board: CSV out at 230400
#define ESP_LINK_VISION 1 // TestBlobs2.0 board: V lines in at 57600

#define ESP_LINK_MODE   ESP_LINK_VISION
