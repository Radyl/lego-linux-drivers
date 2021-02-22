
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>

#include "modbrick.h"
#include "../sensors/ev3_analog_sensor.h"
#include "../sensors/ev3_uart_sensor.h"
#include "../sensors/nxt_analog_sensor.h"
#include "../sensors/nxt_i2c_sensor.h"

struct modbrick_in_port {
	struct modbrick_dev *modbrick;
	struct lego_port_device port;
	struct work_struct poll_work;
	struct hrtimer poll_timer;
	struct lego_device *sensor;
	enum modbrick_input_port index;
};

const struct device_type modbrick_in_port_type = {
	.name   = "modbrick-in-port",
};
EXPORT_SYMBOL_GPL(modbrick_in_port_type);

static const struct device_type modbrick_in_port_device_types[NUM_MODBRICK_IN_PORT_MODES] = {
	[MODBRICK_IN_PORT_MODE_NONE] = {
		.name = NULL,
	},
	[MODBRICK_IN_PORT_MODE_NXT_ANALOG] = {
		.name = "nxt-analog-sensor",
	},
	[MODBRICK_IN_PORT_MODE_NXT_COLOR] = {
		.name = "nxt-color-sensor",
	},
	[MODBRICK_IN_PORT_MODE_NXT_I2C] = {
		.name = "nxt-i2c-sensor",
	},
	[MODBRICK_IN_PORT_MODE_EV3_ANALOG] = {
		.name = "ev3-analog-sensor",
	},
	[MODBRICK_IN_PORT_MODE_EV3_UART] = {
		.name = "ev3-uart-sensor",
	},
};

static const struct lego_port_mode_info modbrick_in_port_mode_info[NUM_MODBRICK_IN_PORT_MODES] = {

	[MODBRICK_IN_PORT_MODE_NONE] = {
		.name	= "none",
	},
	[MODBRICK_IN_PORT_MODE_NXT_ANALOG] = {
		.name	= "nxt-analog",
	},
	[MODBRICK_IN_PORT_MODE_NXT_COLOR] = {
		.name	= "nxt-color",
	},
	[MODBRICK_IN_PORT_MODE_NXT_I2C] = {
		.name	= "nxt-i2c",
	},
	[MODBRICK_IN_PORT_MODE_EV3_ANALOG] = {
		.name	= "ev3-analog",
	},
	[MODBRICK_IN_PORT_MODE_EV3_UART] = {
		.name	= "ev3-uart",
	},
};

static void modbrick_in_port_poll_work(struct work_struct *work)
{
	struct modbrick_in_port *data = container_of(work, struct modbrick_in_port, poll_work);
	u8 *raw_data = data->port.raw_data;
	int ret;

    if (raw_data) {
        // Writes data to raw_data (16 bytes)
        ret = ModBrick_Sensor_GetData(data->modbrick, data->index, raw_data);
        if(ret < 0)
            return;
		lego_port_call_raw_data_func(&data->port);
	}
}

static enum hrtimer_restart modbrick_in_port_poll_timer_function(struct hrtimer *timer)
{
	struct modbrick_in_port *data = container_of(timer, struct modbrick_in_port, poll_timer);
    // TODO: Check CPU loads with different periods
	hrtimer_forward_now(&data->poll_timer, ms_to_ktime(2));
	schedule_work(&data->poll_work);
	return HRTIMER_RESTART;
}

static int modbrick_in_port_register_sensor(struct modbrick_in_port *data, const struct device_type *device_type,const char *name)
{
    struct lego_device *new_sensor;

    new_sensor = lego_device_register(name, device_type, &data->port, NULL, 0);

    if (IS_ERR(new_sensor))
        return PTR_ERR(new_sensor);

    data->sensor = new_sensor;
    hrtimer_start(&data->poll_timer, ms_to_ktime(2), HRTIMER_MODE_REL);

	return 0;
}

static void modbrick_in_port_unregister_sensor(struct modbrick_in_port *data)
{
	if (data->sensor) {
		hrtimer_cancel(&data->poll_timer);
		cancel_work_sync(&data->poll_work);
		lego_device_unregister(data->sensor);
		data->sensor = NULL;
	}
}











static int modbrick_in_port_set_device(void *context, const char *name)
{
	struct modbrick_in_port *data = context;
	int mode = data->port.mode;

	modbrick_in_port_unregister_sensor(data);

	if (mode == MODBRICK_IN_PORT_MODE_NONE)
		return -EOPNOTSUPP;
    
	return modbrick_in_port_register_sensor(data, &modbrick_in_port_device_types[mode], name);
}

static int modbrick_in_port_set_mode(void *context, u8 mode)
{
	struct modbrick_in_port *data = context;
	const char *name = NULL;
	int ret;

	modbrick_in_port_unregister_sensor(data);
	
    
    switch (mode) {
	case MODBRICK_IN_PORT_MODE_NONE:
        break;
	case MODBRICK_IN_PORT_MODE_EV3_UART:
        break;
	case MODBRICK_IN_PORT_MODE_NXT_ANALOG:
		name = GENERIC_NXT_ANALOG_SENSOR_NAME;
		break;
	case MODBRICK_IN_PORT_MODE_EV3_ANALOG:
		name = LEGO_EV3_TOUCH_SENSOR_NAME;
		break;
    case MODBRICK_IN_PORT_MODE_NXT_I2C:
        return -EOPNOTSUPP;
        break;
	default:
        return -EOPNOTSUPP;
        break;
    }

    // Update sensor mode
    ret = ModBrick_Sensor_SetMode(data->modbrick, data->index, mode);

	if (!name)
		return ret;

	return modbrick_in_port_register_sensor(data, &modbrick_in_port_device_types[mode], name);
}

static int modbrick_in_port_set_pin5_gpio(void *context, enum lego_port_gpio_state state)
{
	struct modbrick_in_port *data = context;
  	int ret;
	ret = ModBrick_Sensor_NxtAnalogSetPin5(data->modbrick, data->index, state==LEGO_PORT_GPIO_HIGH ? 1 : 0);
    	return ret; 
}

static struct lego_port_nxt_analog_ops modbrick_in_port_nxt_analog_ops = {
	.set_pin5_gpio = modbrick_in_port_set_pin5_gpio,
};

static int modbrick_in_port_set_ev3_uart_sensor_mode(void *context, u8 type_id,	u8 mode)
{
	struct modbrick_in_port *data = context;
    
	return ModBrick_Sensor_Ev3UartSetMode(data->modbrick, data->index, mode);
}

static const struct lego_port_ev3_uart_ops modbrick_ev3_uart_ops = {
	.set_mode = modbrick_in_port_set_ev3_uart_sensor_mode,
};

static void modbrick_ports_in_release(struct device *dev, void *res)
{
	struct modbrick_in_port *data = res;

	modbrick_in_port_unregister_sensor(data);
	lego_port_unregister(&data->port);
}

static int devm_modbrick_port_in_register_one(struct device *dev, struct modbrick_dev *modbrick, enum modbrick_input_port port)
{
	struct modbrick_in_port *data;
	int ret;

	data = devres_alloc(modbrick_ports_in_release, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->modbrick = modbrick;
	data->index = port;
	INIT_WORK(&data->poll_work, modbrick_in_port_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = modbrick_in_port_poll_timer_function;

	data->port.name = modbrick_in_port_type.name;
	snprintf(data->port.address, LEGO_NAME_SIZE, "%s:in%d", dev_name(dev), port + 1);
	data->port.num_modes = NUM_MODBRICK_IN_PORT_MODES;
	data->port.supported_modes = LEGO_PORT_ALL_MODES;
	data->port.mode_info = modbrick_in_port_mode_info;
	data->port.set_mode = modbrick_in_port_set_mode;
	data->port.set_device = modbrick_in_port_set_device;
	data->port.context = data;
	data->port.nxt_analog_ops = &modbrick_in_port_nxt_analog_ops;
	data->port.ev3_uart_ops = &modbrick_ev3_uart_ops;

	ret = lego_port_register(&data->port, &modbrick_in_port_type, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register input port %s\n",
			data->port.address);
		return ret;
	}

	devres_add(dev, data);

	return 0;
}

int devm_modbrick_register_in_ports(struct device *dev, struct modbrick_dev *modbrick)
{
	int i, ret;

	for (i = 0; i < NUM_MODBRICK_IN_PORTS; i++) {
		ret = devm_modbrick_port_in_register_one(dev, modbrick, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}
