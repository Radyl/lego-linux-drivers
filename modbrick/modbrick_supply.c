

#include <linux/device.h>
#include <linux/module.h>
#include <linux/power_supply.h>

#include "modbrick.h"






struct modbrick_supply{   
    struct modbrick_dev *modbrick;
    struct power_supply *supply;
};

static enum power_supply_property modbrick_supply_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

static int modbrick_supply_get_property(struct power_supply *supply, enum power_supply_property property, union power_supply_propval *val)
{
	struct modbrick_supply *data = power_supply_get_drvdata(supply);
    int ret = 0;
    uint16_t volatage_mV;

	switch (property) {
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            ret = ModBrick_Supply_ReadVoltage(data->modbrick, 0, &volatage_mV);
            if(ret < 0)
                return EIO;
            val->intval = 1000 * volatage_mV;
            break;
        case POWER_SUPPLY_PROP_SCOPE:
            val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
            break;
        default:
            return -EINVAL;
	}

	return 0;
}


static const struct power_supply_desc modbrick_supply_desc = {
	.name			= "modbrick-supply",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= modbrick_supply_get_property,
	.properties		= modbrick_supply_props,
	.num_properties	= ARRAY_SIZE(modbrick_supply_props),
};






static void modbrick_supply_release(struct device *dev, void *res)
{
	struct modbrick_supply *data = res;

	power_supply_unregister(data->supply);
}


int devm_modbrick_register_supply(struct device *dev, struct modbrick_dev *modbrick ){
    
    struct modbrick_supply *data;
    struct power_supply_config supply_config = {};
    
    data = devres_alloc(modbrick_supply_release, sizeof(*data), GFP_KERNEL);
    if(!data)
        return -ENOMEM;
    
    supply_config.drv_data = data;
    
    data->modbrick = modbrick;
    data->supply = power_supply_register(dev, &modbrick_supply_desc, &supply_config);
	if (IS_ERR(data->supply)) {
		dev_err(dev, "Failed to register power supply\n");
		return PTR_ERR(data->supply);
	}
    
    devres_add(dev, data);
    
    return 0;
}