#include <stdint.h>
#include <stdlib.h>
#include "comm.h"
#include "scheduler.h"

static MotorMovement motorMovement[NUM_MOTORS];
static char hardStop;

static int applyAcceleration( Accelerating_t *accelerating, char firstCommand );
static int applyConstantSpeed( ConstantSpeed_t *constantSpeed );
static int applyHome( ConstantSpeed_t *constantSpeed );
static int applyWorkHead( WorkHead_t *work );

int schedulerInit( void ) {
    int i;
    for( i = 0; i < NUM_MOTORS; ++i ) {
        motorMovement[i].steps = 0;
        motorMovement[i].fractionalStep = 0;
        motorMovement[i].speed = 0; // (2^32)/50k/desired freq
        motorMovement[i].acceleration = 0;
    }

    return 1;
}

int updateMotors( void ) {
    MotorMovement *motor;
    int getNextCommand = 1;
    int i;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motor = &motorMovement[i];
        if( motor->steps > 0 ) {
            getNextCommand = 0;
        } else if( hardStop ) {
            continue;
        }
        motor->speed += motor->acceleration;
        #ifndef x86
        asm( " sb clearVflag cond nov" );
        asm( "clearVflag" );
        #endif
        motor->fractionalStep += motor->speed;
        #ifdef x86
        asm( "jno noOverflow" );
        #else
        asm( " sb noOverflow cond nov" );
        #endif
        if( !moveMotor( i ) ) {
            return 0;
        }
        motor->steps--;
        asm( "noOverflow:" );
    }
    if( getNextCommand ) {
        return -1;
    }
    return 1;
}

int applyCommand( Command_t *command, char commandCount ) {
    if( commandCount == 3 ) {
        hardStop = 1;
    } else {
        hardStop = 0;
    }
    switch( command->commandType & 0x000000FF ) {
        case Accelerating:
            return applyAcceleration( &command->command.accelerating, commandCount == 0 );
        case ConstantSpeed:
            return applyConstantSpeed( &command->command.constantSpeed );
        case WorkHead:
            return applyWorkHead( &command->command.workHead );
        case Home:
            return applyHome( &command->command.constantSpeed );
        default:
            return 0;
    }
}

static int applyWorkHead( WorkHead_t *work ) {
    setDirection( 3, sign( work->direction ) );
    setWorkHead( work->dutyCycle );
    return 1;
}

static int applyAcceleration( Accelerating_t *accelerating, char firstCommand ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        if( firstCommand ) {
            setDirection( i, sign( accelerating->accelerations[i] ) );
        }
        motorMovement[i].steps += accelerating->steps[i];
        motorMovement[i].acceleration = accelerating->accelerations[i];
    }
    return 1;
}

static int applyConstantSpeed( ConstantSpeed_t *constantSpeed ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        motorMovement[i].steps += constantSpeed->steps[i];
        motorMovement[i].acceleration = 0;
        motorMovement[i].speed = constantSpeed->speeds[i];
    }
    return 1;
}

static int applyHome( ConstantSpeed_t *constantSpeed ) {
    int motorNumber;

    applyConstantSpeed( constantSpeed );

    for( motorNumber = 0; motorNumber < NUM_MOTORS; motorNumber++ ) {
        if( constantSpeed->steps[motorNumber] ) {
            break;
        }
    }

    while( !isHomed( motorNumber ) ) {
        if( listenForShutdown() ) {
            return 0;
        }
    }
    motorMovement[motorNumber].steps = 0;

    return 1;
}

