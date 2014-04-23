#define sign(x) ( x >= 0 )

typedef struct MotorMovement MotorMovement;

struct MotorMovement {
    int32_t steps;
    int32_t fractionalStep;
    int32_t speed;
    int32_t acceleration;
};

extern int listenForShutdown( void );
extern int schedulerInit( void );
extern int applyCommand( Command_t *command, char commandCount );
extern int updateMotors( void );
extern int moveMotor( int motorNumber );
extern char isHomed( int motorNumber );
extern int setDirection( int motorNumber, int forwardDirection );
extern int setWorkHead( int dutyCycle );
