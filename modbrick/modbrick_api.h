
#ifndef _MODBRICK_API_H_
#define _MODBRICK_API_H_

#define API_VERSION                   ((uint8_t) 0x01 )

#define API_HEADER_SIGNATURE          ((uint8_t) 0xAC )
#define API_RESPONSE_SIGNATURE        ((uint8_t) 0xCA )
#define API_MSG_MAX_BYTES             ( 64 )

#define NUM_FUNCTIONS                     ((uint8_t) 17)
#define MODBRICK_SYSTEM_GETINFO           ((uint8_t) 0)
#define MODBRICK_MOTOR_GETINFO            ((uint8_t) 1)
#define MODBRICK_MOTOR_SETPOSITION        ((uint8_t) 2)
#define MODBRICK_MOTOR_RUN_UNREGULATED    ((uint8_t) 3)
#define MODBRICK_MOTOR_RUN_REGULATED      ((uint8_t) 4)
#define MODBRICK_MOTOR_RUN_TOPOSITION     ((uint8_t) 5)
#define MODBRICK_MOTOR_STOP               ((uint8_t) 6)
#define MODBRICK_MOTOR_RESET              ((uint8_t) 7)
#define MODBRICK_SENSOR_SETMODE           ((uint8_t) 8)
#define MODBRICK_SENSOR_EV3UARTSETMODE    ((uint8_t) 9)
#define MODBRICK_SENSOR_NXTANALOGSETPIN5  ((uint8_t) 10)
#define MODBRICK_SENSOR_GETDATA           ((uint8_t) 11)
#define MODBRICK_LED_WRITE                ((uint8_t) 12)
#define MODBRICK_SPEAKER_PLAYSOUND        ((uint8_t) 13)
#define MODBRICK_SPEAKER_STOPSOUND        ((uint8_t) 14)
#define MODBRICK_SPEAKER_GETSTATUS        ((uint8_t) 15)
#define MODBRICK_SUPPLY_READVOLTAGE       ((uint8_t) 16)


#endif


