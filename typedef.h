#ifndef TYPEDEF_H
#define TYPEDEF_H
#define M_PI 3.14159267
#define DEFAULT_IMAGE_WIDTH 320
#define DEFAULT_IMAGE_HEIGHT 240
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned int uint32_t;
typedef int int32_t;
typedef unsigned long long uint64_t;


typedef enum{
    LT_BUMP_MASK_NON                        = 0,
    LT_BUMP_MASK_LEFT                       = (1 << 0),
    LT_BUMP_MASK_FRONT_LEFT                 = (1 << 1),
    LT_BUMP_MASK_FRONT_CENTER_LEFT          = (1 << 2),
    LT_BUMP_MASK_FRONT_CENTER_RIGHT         = (1 << 3),
    LT_BUMP_MASK_FRONT_RIGHT                = (1 << 4),
    LT_BUMP_MASK_RIGHT                      = (1 << 5),
} LTBumpMask;

typedef enum{
    CLIFF_MASK_NONE = 0,

    CLIFF_MASK_SIDE_LEFT        = (1 << 0),
    CLIFF_MASK_FRONT_LEFT       = (1 << 1),
    CLIFF_MASK_FRONT_CENTER     = (1 << 2),
    CLIFF_MASK_FRONT_RIGHT      = (1 << 3),
    CLIFF_MASK_SIDE_RIGHT       = (1 << 4),
    CLIFF_MASK_BACK_RIGHT       = (1 << 5),
    CLIFF_MASK_BACK_CENTER      = (1 << 6),
    CLIFF_MASK_BACK_LEFT        = (1 << 7),

    CLIFF_MASK_FRONT = (CLIFF_MASK_FRONT_LEFT | CLIFF_MASK_FRONT_CENTER | CLIFF_MASK_FRONT_RIGHT),
    CLIFF_MASK_BACK = (CLIFF_MASK_BACK_LEFT | CLIFF_MASK_BACK_CENTER | CLIFF_MASK_BACK_RIGHT)
}CliffMask;

enum CLIFF_TYPE
{
    CLIFF_CENTER = 0,
    CLIFF_LEFT,
    CLIFT_RIGHT,
    CLIFF_MAX,
};
enum LT_TYPE
{
    LT_CENTER = 0,
    LT_FRONT_LEFT,
    LT_LEFT,
    LT_FRONT_RIGHT,
    LT_RIGHT,
    LT_MAX,
};

typedef struct _robot_cliff_t
{
    uint32_t    timestamp;
    uint8_t     is_probable;    ///< Cliff detected but not 100% that it is real
    uint8_t     is_certain;     ///< Cliff detected and confirmed
    CliffMask   mask;
    float       direction;
} robot_cliff_t;

typedef struct _ir_range_sensor_t
    {
        uint16_t right;
        uint16_t front_right;
        uint16_t front_center;
        uint16_t front_left;
        uint16_t left;
        uint64_t timestamp;
    } robot_ir_range_sensor_t;
#endif // TYPEDEF_H
