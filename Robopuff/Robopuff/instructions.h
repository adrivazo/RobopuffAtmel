//
//  instructions.h
//  robopuff
//
//  Created by Adriana Catalina Vazquez Ortiz on 11/18/14.
//  Copyright (c) 2014 adrivazo. All rights reserved.
//

#ifndef __robopuff_instructions__
#define __robopuff_instructions__
#include <stdio.h>


/************************************************************************/
/* Instructions - The game controller will send your robot instructions in a 10-byte packet, as described below.
Comm Test	0xA0	0xA0	0xA0	0xA0	0xA0	0xA0	0xA0	0xA0	0xA0	0xA0	robots must flash the positioning LED*
Play		0xA1	0xA1	0xA1	0xA1	0xA1	0xA1	0xA1	0xA1	0xA1	0xA1	robots must illuminate the positioning LED and move in a noticeable way
Goal A		0xA2	SA		SB		0		0		0		0		0		0		0		SA = score A, SB = score B
Goal B		0xA3	SA		SB		0		0		0		0		0		0		0		SA = score A, SB = score B
Pause		0xA4	0xA4	0xA4	0xA4	0xA4	0xA4	0xA4	0xA4	0xA4	0xA4	robots must stop within three seconds
Halftime	0xA6	0xA6	0xA6	0xA6	0xA6	0xA6	0xA6	0xA6	0xA6	0xA6
Game Over	0xA7	0xA7	0xA7	0xA7	0xA7	0xA7	0xA7	0xA7	0xA7	0xA7
Enemy Pos	0xA8	A1	X1	Y1	A2	X2	Y2	A3	X3	Y3	signed char centimeters from rink center                                                                     */
/************************************************************************/



typedef enum {COMM_TEST, PLAY, GOAL_A, GOAL_B, PAUSE, HALF_TIME,GAME_OVER, ENEMY_POSITION} instructions_t;


#endif /* __robopuff_instructions__ */
