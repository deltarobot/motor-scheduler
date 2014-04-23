#define NUM_MOTORS 4
#define NUM_WORK_HEADS 1

enum CommandType {
    NoOp = 0,
    ConstantSpeed = 1,
    Accelerating = 2,
    Home = 3,
    ReverseHome = 4,
    WorkHead = 5
};

typedef struct Accelerating_t Accelerating_t;
typedef struct ConstantSpeed_t ConstantSpeed_t;
typedef struct WorkHead_t WorkHead_t;
typedef struct Command_t Command_t;

struct Accelerating_t {
    int32_t steps[NUM_MOTORS];
    int32_t accelerations[NUM_MOTORS];
};

struct ConstantSpeed_t {
    int32_t steps[NUM_MOTORS];
    int32_t speeds[NUM_MOTORS];
};

struct WorkHead_t {
    int32_t dutyCycle;
    int32_t direction;
};

struct Command_t {
    int32_t commandType;
    union {
        Accelerating_t accelerating;
        ConstantSpeed_t constantSpeed;
        WorkHead_t workHead;
    } command;
};
