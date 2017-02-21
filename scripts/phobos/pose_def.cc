/*
 * This file is used to define a pose type that was used prior to commit
 * 4f0441483b984865d13581929d681392a4ac42db
 */

struct __attribute__((__packed__)) pose_t {
    float x; /* m */
    float y; /* m */
    float pitch; /* rad */
    float yaw; /* rad */
    float roll; /* rad */
    float steer; /* rad */
    float rear_wheel; /* rad */
    float v; /* m/s */
    uint8_t timestamp;  /* Converted from system ticks to microseconds */
}; /* 33 bytes */
