/*
 * ModBrick Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/string.h>

#include "modbrick.h"
#include "modbrick_api.h"


// These macros will place the given value inside the buffer, so that the Raspberry Pi
// will receive it in the correct byte order (little endian)
#define SPI_STORE_32(buf, base, val) \
    *(buf+base+0) = (uint8_t) ((uint32_t)(val) >> 0U); \
    *(buf+base+1) = (uint8_t) ((uint32_t)(val) >> 8U); \
    *(buf+base+2) = (uint8_t) ((uint32_t)(val) >> 16U); \
    *(buf+base+3) = (uint8_t) ((uint32_t)(val) >> 24U)
#define SPI_STORE_16(buf, base, val) \
    *(buf+base+0) = (uint8_t) ((uint16_t)(val) >> 0U); \
    *(buf+base+1) = (uint8_t) ((uint16_t)(val) >> 8U)
#define SPI_STORE_8(buf, base, val) \
    *(buf+base+0) = (uint8_t) ((uint8_t)(val) >> 0U)

#define SPI_LOAD_32(buf, base) ( \
     ( ( *(buf+base+0) & 0xFFU) << 0U ) | \
     ( ( *(buf+base+1) & 0xFFU) << 8U ) | \
     ( ( *(buf+base+2) & 0xFFU) << 16U ) | \
     ( ( *(buf+base+3) & 0xFFU) << 24U ) )
#define SPI_LOAD_16(buf, base) ( \
     ( ( *(buf+base+0) & 0xFFU) << 0U ) | \
     ( ( *(buf+base+1) & 0xFFU) << 8U ) )
#define SPI_LOAD_8(buf, base) ( \
     ( ( *(buf+base+0) & 0xFFU) << 0U ) )


struct modbrick_dev {
	struct spi_device *spi;
	u8 buf[API_MSG_MAX_BYTES];
	struct spi_message msg;
	struct spi_transfer xfer;
	struct mutex xfer_lock;
};


int ModBrick_System_GetInfo       (struct modbrick_dev * modbrick, uint8_t *software_version, uint8_t *hardware_version) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SYSTEM_GETINFO);
    modbrick->xfer.len = 28; // 23

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SYSTEM_GETINFO, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
    if(software_version) *software_version = SPI_LOAD_8(modbrick->buf, 21);
    if(hardware_version) *hardware_version = SPI_LOAD_8(modbrick->buf, 22);
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_GetInfo        (struct modbrick_dev * modbrick, uint8_t motor_port, uint8_t *motor_status, int16_t *motor_pwm, int16_t *motor_speed, int32_t *motor_position) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_GETINFO);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    modbrick->xfer.len = 30;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_GETINFO, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
    if(motor_status   ) *motor_status    = SPI_LOAD_8(modbrick->buf, 21);
    if(motor_pwm      ) *motor_pwm       = SPI_LOAD_16(modbrick->buf, 22);
    if(motor_speed    ) *motor_speed     = SPI_LOAD_16(modbrick->buf, 24);
    if(motor_position ) *motor_position  = SPI_LOAD_32(modbrick->buf, 26);
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_SetPosition    (struct modbrick_dev * modbrick, uint8_t motor_port, int32_t encoder_value) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_SETPOSITION);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    SPI_STORE_32(modbrick->buf, 3, encoder_value);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_SETPOSITION, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_Run_Unregulated(struct modbrick_dev * modbrick, uint8_t motor_port, int16_t pwm) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_RUN_UNREGULATED);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    SPI_STORE_16(modbrick->buf, 3, pwm);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_RUN_UNREGULATED, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_Run_Regulated  (struct modbrick_dev * modbrick, uint8_t motor_port, int16_t speed) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_RUN_REGULATED);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    SPI_STORE_16(modbrick->buf, 3, speed);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_RUN_REGULATED, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_Run_ToPosition (struct modbrick_dev * modbrick, uint8_t motor_port, int16_t speed, int32_t position) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_RUN_TOPOSITION);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    SPI_STORE_16(modbrick->buf, 3, speed);
    SPI_STORE_32(modbrick->buf, 5, position);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_RUN_TOPOSITION, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_Stop           (struct modbrick_dev * modbrick, uint8_t motor_port, uint8_t stop_action) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_STOP);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    SPI_STORE_8(modbrick->buf, 3, stop_action);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_STOP, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Motor_Reset          (struct modbrick_dev * modbrick, uint8_t motor_port) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_MOTOR_RESET);
    SPI_STORE_8(modbrick->buf, 2, motor_port);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_MOTOR_RESET, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Sensor_SetMode       (struct modbrick_dev * modbrick, uint8_t sensor_port, uint8_t sensor_type) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SENSOR_SETMODE);
    SPI_STORE_8(modbrick->buf, 2, sensor_port);
    SPI_STORE_8(modbrick->buf, 3, sensor_type);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SENSOR_SETMODE, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Sensor_Ev3UartSetMode(struct modbrick_dev * modbrick, uint8_t sensor_port, uint8_t mode) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SENSOR_EV3UARTSETMODE);
    SPI_STORE_8(modbrick->buf, 2, sensor_port);
    SPI_STORE_8(modbrick->buf, 3, mode);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SENSOR_EV3UARTSETMODE, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Sensor_NxtAnalogSetPin5(struct modbrick_dev * modbrick, uint8_t sensor_port, uint8_t level) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SENSOR_NXTANALOGSETPIN5);
    SPI_STORE_8(modbrick->buf, 2, sensor_port);
    SPI_STORE_8(modbrick->buf, 3, level);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SENSOR_NXTANALOGSETPIN5, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Sensor_GetData       (struct modbrick_dev * modbrick, uint8_t sensor_port, void *data) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SENSOR_GETDATA);
    SPI_STORE_8(modbrick->buf, 2, sensor_port);
    modbrick->xfer.len = 37;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SENSOR_GETDATA, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
    if(data)
        memcpy(data, &modbrick->buf[21], (size_t)16 );
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_LED_Write            (struct modbrick_dev * modbrick, uint8_t nr, uint8_t brightness) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_LED_WRITE);
    SPI_STORE_8(modbrick->buf, 2, nr);
    SPI_STORE_8(modbrick->buf, 3, brightness);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_LED_WRITE, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Speaker_PlaySound    (struct modbrick_dev * modbrick, uint16_t frequency, uint16_t time_ms) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SPEAKER_PLAYSOUND);
    SPI_STORE_16(modbrick->buf, 2, frequency);
    SPI_STORE_16(modbrick->buf, 4, time_ms);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SPEAKER_PLAYSOUND, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Speaker_StopSound    (struct modbrick_dev * modbrick) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SPEAKER_STOPSOUND);
    modbrick->xfer.len = 21;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SPEAKER_STOPSOUND, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Speaker_GetStatus    (struct modbrick_dev * modbrick, uint8_t *status) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SPEAKER_GETSTATUS);
    modbrick->xfer.len = 22;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SPEAKER_GETSTATUS, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
    if(status         ) *status          = SPI_LOAD_8(modbrick->buf, 21);
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}

int ModBrick_Supply_ReadVoltage   (struct modbrick_dev * modbrick, uint8_t voltage_type, uint16_t *voltage) {
    int ret = 0;

    mutex_lock(&modbrick->xfer_lock);

    SPI_STORE_8(modbrick->buf, 0, 0xAC);
    SPI_STORE_8(modbrick->buf, 1, MODBRICK_SUPPLY_READVOLTAGE);
    SPI_STORE_8(modbrick->buf, 2, voltage_type);
    modbrick->xfer.len = 23;

    ret = spi_sync(modbrick->spi, &modbrick->msg);
    if (ret < 0 || modbrick->buf[19] != 0xCA) {
        printk("ModBrick: SPI-Error: %d, %d, %d, %d", MODBRICK_SUPPLY_READVOLTAGE, ret, modbrick->buf[19], modbrick->buf[20]);
        ret = -EIO;
        goto out;
    } else if (modbrick->buf[20] != 0) {
        ret = -modbrick->buf[20];
        goto out;
    }
    if(voltage        ) *voltage         = SPI_LOAD_16(modbrick->buf, 21);
out:
    mutex_unlock(&modbrick->xfer_lock);
    return ret;
}



static int modbrick_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct modbrick_dev *modbrick;
    uint8_t sw_ver, hw_ver;
	int ret;

	modbrick = devm_kzalloc(dev, sizeof(*modbrick), GFP_KERNEL);
	if (!modbrick)
		return -ENOMEM;

	dev_set_drvdata(dev, modbrick);

	modbrick->spi = spi;
	modbrick->xfer.bits_per_word = 8;
	modbrick->xfer.tx_buf = modbrick->buf;
	modbrick->xfer.rx_buf = modbrick->buf;
	spi_message_init_with_transfers(&modbrick->msg, &modbrick->xfer, 1);
	mutex_init(&modbrick->xfer_lock);
    
    // Board and API detection
    ret = ModBrick_System_GetInfo(modbrick, &sw_ver, &hw_ver );
    if (ret < 0){
        printk("ModBrick: Could not communicate with board");
        return -ENODEV;
    }
    if (sw_ver != API_VERSION){
        printk("ModBrick: Driver only compatible with version %d, but was %d (%d)", API_VERSION, sw_ver, hw_ver);
        return -EPROTO;
    }
    
    ret = devm_modbrick_register_board(dev, modbrick);
    if (ret < 0)
        return ret;
    
    ret = devm_modbrick_register_supply(dev, modbrick);
    if (ret < 0)
        return ret;
    
    ret = devm_modbrick_register_leds(dev, modbrick);
	if (ret < 0)
		return ret;
    
	ret = devm_modbrick_register_out_ports(dev, modbrick);
	if (ret < 0)
		return ret;
    
    ret = devm_modbrick_register_in_ports(dev, modbrick);
	if (ret < 0)
		return ret;
	
    printk("ModBrick: Board successfully initialized ( SW: %d, HW: %d )", sw_ver, hw_ver);
        
	return 0;
}

const static struct of_device_id modbrick_of_match_table[] = {
	{ .compatible = "brick,modbrick", },
	{ }
};
MODULE_DEVICE_TABLE(of, modbrick_of_match_table);

static struct spi_driver modbrick_driver = {
	.driver	= {
		.name		= "modbrick",
		.of_match_table	= modbrick_of_match_table,
	},
	.probe	= modbrick_probe,
};
module_spi_driver(modbrick_driver);

MODULE_DESCRIPTION("ModBrick");
MODULE_AUTHOR("Robert Stiegler <robert.stiegler2@tu-dresden.de>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:modbrick");
