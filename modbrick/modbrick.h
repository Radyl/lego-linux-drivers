

#ifndef _MODBRICK_H_
#define _MODBRICK_H_

enum modbrick_input_port {
	MODBRICK_IN1 = 0,
	MODBRICK_IN2,
	MODBRICK_IN3,
	MODBRICK_IN4,
	NUM_MODBRICK_IN_PORTS
};

enum modbrick_output_port {
	MODBRICK_OUTA = 0,
	MODBRICK_OUTB,
	MODBRICK_OUTC,
	MODBRICK_OUTD,
	NUM_MODBRICK_OUT_PORTS
};

enum modbrick_in_port_mode {
	MODBRICK_IN_PORT_MODE_NONE,
	MODBRICK_IN_PORT_MODE_NXT_ANALOG,
	MODBRICK_IN_PORT_MODE_NXT_COLOR,
	MODBRICK_IN_PORT_MODE_NXT_I2C,
	MODBRICK_IN_PORT_MODE_EV3_ANALOG,
	MODBRICK_IN_PORT_MODE_EV3_UART,
	NUM_MODBRICK_IN_PORT_MODES
};

enum modbrick_out_port_mode {
	MODBRICK_OUT_PORT_MODE_TACHO_MOTOR,
	MODBRICK_OUT_PORT_MODE_DC_MOTOR,
	MODBRICK_OUT_PORT_MODE_LED,
	NUM_MODBRICK_OUT_PORT_MODES
};

struct modbrick_dev;



int ModBrick_System_GetInfo       (struct modbrick_dev * modbrick, uint8_t *software_version, uint8_t *hardware_version);
int ModBrick_Motor_GetInfo        (struct modbrick_dev * modbrick, uint8_t motor_port, uint8_t *motor_status, int16_t *motor_pwm, int16_t *motor_speed, int32_t *motor_position);
int ModBrick_Motor_SetPosition    (struct modbrick_dev * modbrick, uint8_t motor_port, int32_t encoder_value);
int ModBrick_Motor_Run_Unregulated(struct modbrick_dev * modbrick, uint8_t motor_port, int16_t pwm);
int ModBrick_Motor_Run_Regulated  (struct modbrick_dev * modbrick, uint8_t motor_port, int16_t speed);
int ModBrick_Motor_Run_ToPosition (struct modbrick_dev * modbrick, uint8_t motor_port, int16_t speed, int32_t position);
int ModBrick_Motor_Stop           (struct modbrick_dev * modbrick, uint8_t motor_port, uint8_t stop_action);
int ModBrick_Motor_Reset          (struct modbrick_dev * modbrick, uint8_t motor_port);
int ModBrick_Sensor_SetMode       (struct modbrick_dev * modbrick, uint8_t sensor_port, uint8_t sensor_type);
int ModBrick_Sensor_Ev3UartSetMode(struct modbrick_dev * modbrick, uint8_t sensor_port, uint8_t mode);
int ModBrick_Sensor_NxtAnalogSetPin5(struct modbrick_dev * modbrick, uint8_t sensor_port, uint8_t level);
int ModBrick_Sensor_GetData       (struct modbrick_dev * modbrick, uint8_t sensor_port, void *data);
int ModBrick_LED_Write            (struct modbrick_dev * modbrick, uint8_t nr, uint8_t brightness);
int ModBrick_Speaker_PlaySound    (struct modbrick_dev * modbrick, uint16_t frequency, uint16_t time_ms);
int ModBrick_Speaker_StopSound    (struct modbrick_dev * modbrick);
int ModBrick_Speaker_GetStatus    (struct modbrick_dev * modbrick, uint8_t *status);
int ModBrick_Supply_ReadVoltage   (struct modbrick_dev * modbrick, uint8_t voltage_type, uint16_t *voltage);

int devm_modbrick_register_board(struct device *dev, struct modbrick_dev *modbrick);
int devm_modbrick_register_supply(struct device *dev, struct modbrick_dev *modbrick );
int devm_modbrick_register_leds(struct device *dev, struct modbrick_dev *modbrick);
int devm_modbrick_register_in_ports(struct device *dev, struct modbrick_dev *modbrick);
int devm_modbrick_register_out_ports(struct device *dev, struct modbrick_dev *modbrick);

#endif /* _MODBRICK_H_ */
