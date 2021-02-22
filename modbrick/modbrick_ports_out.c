

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/workqueue.h>

#include <dc_motor_class.h>
#include <tacho_motor_class.h>

#include "modbrick.h"
#include "../ev3/legoev3_motor.h"

struct modbrick_out_port {
	struct modbrick_dev 		        *modbrick;
	struct lego_port_device 	        port;
	struct lego_device 			        *motor;
	struct delayed_work 		        run_to_pos_work;
	enum tm_stop_action 		        run_to_pos_stop_action;
    enum dc_motor_internal_command      command;
	enum modbrick_output_port 	        index;
    
	s32 						        position_sp;
	s8 							        duty_cycle;
	bool 						        running;
	bool 						        holding;
	bool 						        positioning;
};

const struct device_type modbrick_out_port_type = {
	.name = "modbrick-out-port",
};
EXPORT_SYMBOL_GPL(modbrick_out_port_type);

static const struct device_type
modbrick_out_port_device_types[NUM_MODBRICK_OUT_PORT_MODES] = {
	[MODBRICK_OUT_PORT_MODE_TACHO_MOTOR] = {
		.name = "ev3-motor",
	},
	[MODBRICK_OUT_PORT_MODE_DC_MOTOR] = {
		.name = "rcx-motor",
	},
	[MODBRICK_OUT_PORT_MODE_LED] = {
		.name = "rcx-led",
	},
};

static const char * const
modbrick_out_port_default_driver[NUM_MODBRICK_OUT_PORT_MODES] = {
	[MODBRICK_OUT_PORT_MODE_TACHO_MOTOR]		= "lego-nxt-motor",
	[MODBRICK_OUT_PORT_MODE_DC_MOTOR]			= "rcx-motor",
	[MODBRICK_OUT_PORT_MODE_LED] 				= "rcx-led",
};


static const struct lego_port_mode_info modbrick_out_port_mode_info[NUM_MODBRICK_OUT_PORT_MODES] = {
	[MODBRICK_OUT_PORT_MODE_TACHO_MOTOR] = {
		.name	= "tacho-motor",
	},
	[MODBRICK_OUT_PORT_MODE_DC_MOTOR] = {
		.name	= "dc-motor",
	},
	[MODBRICK_OUT_PORT_MODE_LED] = {
		.name	= "led",
	},
};







static void modbrick_run_to_pos_work(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct modbrick_out_port *data = container_of(delayed_work,
						      struct modbrick_out_port,
						      run_to_pos_work);
	int ret;
	int16_t speed;
	int32_t position;

	/* another command was run, so positioning is canceled */
	if (!data->positioning)
		return;

	ret = ModBrick_Motor_GetInfo(data->modbrick, data->index, NULL, NULL, &speed, &position);

	if (ret < 0) {
		/* let's try again */
		schedule_delayed_work(&data->run_to_pos_work,
				      msecs_to_jiffies(20));
		return;
	}

	if (abs(data->position_sp - position) < 1 && speed == 0) {
		/* we have reached the target position */
		if (data->run_to_pos_stop_action == TM_STOP_ACTION_HOLD) {
			data->running = false;
			data->holding = true;
		} else {
            ModBrick_Motor_Stop(data->modbrick, data->index, data->run_to_pos_stop_action);
		}
	} else  {
		/* keep polling... */
		schedule_delayed_work(&data->run_to_pos_work, msecs_to_jiffies(200));
	}
}






static int modbrick_out_port_get_position(void *context, int *position)
{
	struct modbrick_out_port *data = context;
	int ret;
    
	ret = ModBrick_Motor_GetInfo(data->modbrick, data->index, NULL, NULL, NULL, position);
    if (ret < 0)
		return ret;

    return 0;
}

static int modbrick_out_port_set_position(void *context, int position)
{
	struct modbrick_out_port *data = context;
	int ret;

	return ModBrick_Motor_SetPosition(data->modbrick, data->index, position);
    if (ret < 0)
		return ret;

    return 0;
}

static int modbrick_out_port_run_unregulated(void *context, int duty_cycle)
{
	struct modbrick_out_port *data = context;
	int ret;

    if (data->duty_cycle < 0)
		data->duty_cycle = -duty_cycle;
	else
		data->duty_cycle = duty_cycle;

	ret = ModBrick_Motor_Run_Unregulated(data->modbrick, data->index, (int) (data->duty_cycle * 1023 / 100) );

	if (ret < 0)
		return ret;

	data->running = true;
	data->holding = false;
	data->positioning = false;

	return 0;
}

static int modbrick_out_port_run_regulated(void *context, int speed)
{
	struct modbrick_out_port *data = context;
	int ret;

	ret = ModBrick_Motor_Run_Regulated(data->modbrick, data->index, (int16_t) speed);
	if (ret < 0)
		return ret;

	data->running = true;
	data->holding = false;
	data->positioning = false;

	return 0;
}

static int modbrick_out_port_run_to_pos(void *context, int pos, int speed,
					enum tm_stop_action stop_action)
{
	struct modbrick_out_port *data = context;
	int ret;

	ret = ModBrick_Motor_Run_ToPosition(data->modbrick, data->index, (int16_t) speed, pos);
	if (ret < 0)
		return ret;

	data->run_to_pos_stop_action = stop_action;

	data->position_sp = pos;
	data->running = true;
	data->holding = false;
	data->positioning = true;

	schedule_delayed_work(&data->run_to_pos_work, 0);

	return 0;
}

static int modbrick_out_port_get_state(void *context)
{
	struct modbrick_out_port *data = context;
    unsigned state = 0;
    
    if (data->running)
		state |= BIT(TM_STATE_RUNNING);
	if (data->holding)
		state |= BIT(TM_STATE_HOLDING);

	return state;
}

static int modbrick_out_port_get_duty_cycle(void *context, int *duty_cycle)
{
	struct modbrick_out_port *data = context;
	int ret;
	int16_t dt;
	
	ret = ModBrick_Motor_GetInfo(data->modbrick, data->index, NULL, &dt, NULL, NULL);

	*duty_cycle = dt;
	
	if (ret < 0)
		return ret;
	
	return 0;
}

static unsigned modbrick_out_port_get_duty_cycle_dc(void *context){
    unsigned dt;
    int ret;
    
    ret = modbrick_out_port_get_duty_cycle(context, &dt);
    
    if(ret < 0)
        return 0;
 
    return dt;
}

static int modbrick_out_port_get_speed(void *context, int *speed)
{
	struct modbrick_out_port *data = context;
    int ret;
	
    uint16_t sp;

	ret = ModBrick_Motor_GetInfo( data->modbrick, data->index, NULL, NULL, &sp, NULL);
    
    *speed = sp;

    return ret;
}

static int modbrick_out_port_stop(void *context, enum tm_stop_action stop_action)
{
	struct modbrick_out_port *data = context;
	bool holding = false;
	int ret, pos;

	switch(stop_action) {
	case TM_STOP_ACTION_COAST:
    case TM_STOP_ACTION_BRAKE:
		ret = ModBrick_Motor_Stop(data->modbrick, data->index, stop_action );
		break;
	case TM_STOP_ACTION_HOLD:
		ret = ModBrick_Motor_GetInfo(data->modbrick, data->index, NULL, NULL, NULL, &pos);
		if (ret < 0)
			return ret;
		ret = ModBrick_Motor_Run_ToPosition(data->modbrick, data->index, (int16_t) 10000, pos);
		
		data->position_sp = pos;
		holding = true;
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	data->running = false;
	data->holding = holding;
	data->positioning = false;

	return 0;
}

static int modbrick_out_port_reset(void *context)
{
	struct modbrick_out_port *data = context;
	int ret;

	ret = 0;
	ret = ModBrick_Motor_Reset(data->modbrick, data->index);
	if (ret < 0)
		return ret;

	return 0;
	return modbrick_out_port_set_position(data, 0);
}


static int modbrick_out_port_set_duty_cycle(void *context, unsigned duty_cycle)
{
	return modbrick_out_port_run_unregulated(context, duty_cycle);
}


static enum dc_motor_internal_command modbrick_out_port_get_command(void *context)
{
	struct modbrick_out_port *data = context;
    
    return data->command;
}


static int modbrick_out_port_set_command(void *context, enum dc_motor_internal_command command)
{
	struct modbrick_out_port *data = context;
    int ret;
    
    data->command = command;

	switch(command) {
		case DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD:
            data->duty_cycle = abs(data->duty_cycle);
            // fall through
		case DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE:
			data->duty_cycle = -abs(data->duty_cycle);
            // fall through 
		case DC_MOTOR_INTERNAL_COMMAND_COAST:
			data->duty_cycle = 0;
            ret = ModBrick_Motor_Run_Unregulated(data->modbrick, data->index, data->duty_cycle );
			break;
            
		case DC_MOTOR_INTERNAL_COMMAND_BRAKE:
			data->duty_cycle = 0;
            ret = ModBrick_Motor_Stop(data->modbrick, data->index, TM_STOP_ACTION_BRAKE);
			break;
		default:
			return -EINVAL;
	}
    
    if(ret > 0)
        return ret;
    
    return 0;    
}



static unsigned modbrick_out_port_get_supported_commands(void *context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned modbrick_out_port_get_supported_stop_actions(void *context)
{
	return BIT(DC_MOTOR_STOP_ACTION_COAST) | BIT(DC_MOTOR_STOP_ACTION_BRAKE);
}

static unsigned modbrick_out_port_get_stop_actions(void *context)
{
	return BIT(TM_STOP_ACTION_COAST) |
	       BIT(TM_STOP_ACTION_BRAKE) |
	       BIT(TM_STOP_ACTION_HOLD);
}


static struct dc_motor_ops modbrick_out_port_dc_motor_ops = {
	.get_supported_commands			= modbrick_out_port_get_supported_commands,		
	.get_supported_stop_actions 	= modbrick_out_port_get_supported_stop_actions,	
	.get_command					= modbrick_out_port_get_command,	
	.set_command					= modbrick_out_port_set_command,	
	.set_duty_cycle					= modbrick_out_port_set_duty_cycle,	
	.get_duty_cycle					= modbrick_out_port_get_duty_cycle_dc,
};


struct tacho_motor_ops modbrick_out_port_tacho_motor_ops = {
	.get_position					= modbrick_out_port_get_position,
	.set_position					= modbrick_out_port_set_position,
	.run_unregulated				= modbrick_out_port_run_unregulated,
	.run_regulated					= modbrick_out_port_run_regulated,
	.run_to_pos						= modbrick_out_port_run_to_pos,
	.stop							= modbrick_out_port_stop,
	.reset							= modbrick_out_port_reset,
	.get_state						= modbrick_out_port_get_state,
	.get_duty_cycle					= modbrick_out_port_get_duty_cycle,
	.get_speed						= modbrick_out_port_get_speed,
	.get_stop_actions				= modbrick_out_port_get_stop_actions,
};





static int modbrick_out_port_register_motor(struct modbrick_out_port *out_port, const struct device_type *device_type, const char *name)
{
	struct lego_device *new_motor;

	new_motor = lego_device_register(name, device_type, &out_port->port, NULL, 0);
	if (IS_ERR(new_motor))
		return PTR_ERR(new_motor);
	out_port->motor = new_motor;

	return 0;
}


static void modbrick_out_port_unregister_motor(struct modbrick_out_port *data)
{
	if (data->motor) {
		cancel_delayed_work_sync(&data->run_to_pos_work);
		lego_device_unregister(data->motor);
		data->motor = NULL; 
		ModBrick_Motor_Reset(data->modbrick, data->index );
	}
}

static int modbrick_out_port_set_mode(void *context, u8 mode)
{
	struct modbrick_out_port *data = context;
	
	modbrick_out_port_unregister_motor(data);
	return modbrick_out_port_register_motor(data, &modbrick_out_port_device_types[mode], modbrick_out_port_default_driver[mode]);
}



static void modbrick_out_port_release(struct device *dev, void *res)
{
	struct modbrick_out_port *data = res;

	modbrick_out_port_unregister_motor(data);
	lego_port_unregister(&data->port);
}

static int devm_modbrick_out_port_register_one(struct device *dev, struct modbrick_dev *modbrick, enum modbrick_output_port port)
{
	struct modbrick_out_port *data;
	int ret;

	data = devres_alloc(modbrick_out_port_release, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->modbrick = modbrick; 
	data->index = port;
	data->port.name = modbrick_out_port_type.name;
	snprintf(data->port.address, LEGO_NAME_SIZE, "%s:out%c", dev_name(dev), port + 'A');
	printk("Registering Output Port: %d, %s\n", port, dev_name(dev));
	data->port.num_modes = NUM_MODBRICK_OUT_PORT_MODES;
	data->port.supported_modes = LEGO_PORT_ALL_MODES;
	data->port.mode_info = modbrick_out_port_mode_info;
	data->port.set_mode = modbrick_out_port_set_mode;
	data->port.dc_motor_ops = &modbrick_out_port_dc_motor_ops;
	data->port.tacho_motor_ops = &modbrick_out_port_tacho_motor_ops;
	data->port.context = data;

	INIT_DELAYED_WORK(&data->run_to_pos_work, modbrick_run_to_pos_work);

	ret = lego_port_register(&data->port, &modbrick_out_port_type, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register Modbrick output port OUT%d\n", port);
		return ret;
	}

	devres_add(dev, data);

	ret = modbrick_out_port_set_mode(data, MODBRICK_OUT_PORT_MODE_TACHO_MOTOR);
	if (ret < 0) {
		dev_err(dev, "Failed to set output port mode\n");
		return ret;
	}

	return 0;
}


int devm_modbrick_register_out_ports(struct device *dev, struct modbrick_dev *modbrick) {
	int i, ret;
	
	for (i = 0; i < NUM_MODBRICK_OUT_PORTS; i++) {
		ret = devm_modbrick_out_port_register_one (dev, modbrick, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}
