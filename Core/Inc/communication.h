/*
 * communication.h
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

// communication.h
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"
#include "odometry.h"



// Command definitions
#define CMD_FORWARD    'F'
#define CMD_BACKWARD   'B'
#define CMD_LEFT       'L'
#define CMD_RIGHT      'R'
#define CMD_STOP       'S'

// Message format definitions
#define MSG_HEADER     '$'
#define MSG_FOOTER     '#'

#define MAX_MSG_LEN    16
extern char messageBuffer[MAX_MSG_LEN];

// Extern declarations for global variables
extern volatile uint8_t Rx_data;
extern char messageBuffer[MAX_MSG_LEN];
extern int bufferIndex;
extern int isReceiving;

// Function declarations
void InitCommunication(void);
void ProcessCommand(char* msg);
void SendPWMFeedback(float pwmL, float pwmR);
void ParseCommand(char* msg, char* cmd, int* speed);

#endif



