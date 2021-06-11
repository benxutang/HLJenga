#pragma once

/***************** IP/TCP macro command *****************/
#define ITVMS 100
#define PRSITVMS 50
#define USE_PRESSURE
#define T_OFFSET 0

/***************** motion planning *****************/
#define VEL 0
#define ACC 1
#define DEC 1
#define _Z_ACC 1
#define _TH6_ACC 1

#define T1 1
#define T2 0.4
#define T1_Seg1 1.8
#define T2_Seg1 1.4

#define _HEIGHT_THRESH 60
#define _K_TIME 34

/***************** rect detect *****************/
//#define NEW_MAT_K
#define X_RECT_OFFSET 7
#define Y_RECT_OFFSET -4
#define X_BASE_CENTRE 500
#define Y_BASE_CENTRE 100
#define MAX_RECT_DIST 1000
#define ENTER 13
#define BKSPACE 8

/***************** jenga properties *****************/
#define X_BASE X_BASE_CENTRE
#define Y_BASE (Y_BASE_CENTRE - 25)
#define Z_BASE 456
#define Z_OFFSET 4
#define Z_OFFSET_ACCM 0.3
#define ROLL_VERTICAL 39
#define ROLL_HORIZONTAL (39 -90)
#define SPIRAL_DELTA -180
//#define PATTERN_LINE
//#define PATTERN_CROSS
#define PATTERN_SPIRAL

/***************** main function select *****************/
//#define HECAL
//#define RECTOFFLINE
//#define PLAN 
#define MAIN
//#define NOROBOT