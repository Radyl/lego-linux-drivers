
#include <linux/device.h>

#include "modbrick.h"


#include "../linux/board_info/board_info.h"


#define MODBRICK_BOARD_MODEL_NAME "ModBrick"
#define MODBRICK_BOARD_TYPE BOARD_INFO_TYPE_NAME_AUX


struct modbrick_board_info {
	struct board_info_desc desc;
	char fw_ver[3];
	char hw_ver[3];
};

static const enum board_info_property modbrick_board_properties[] = {
	BOARD_INFO_FW_VER,
	BOARD_INFO_HW_REV,
	BOARD_INFO_MODEL,
	BOARD_INFO_TYPE,
};



static int modbrick_board_get_property(struct board_info *info, enum board_info_property prop, const char **val)
{
	struct modbrick_board_info *data = board_info_get_drvdata(info);
	switch (prop) {
        case BOARD_INFO_FW_VER:
            *val = data->fw_ver;
            break;
        case BOARD_INFO_HW_REV:
            *val = data->hw_ver;
            break;
        case BOARD_INFO_MODEL:
            *val = MODBRICK_BOARD_MODEL_NAME;
            break;
        case BOARD_INFO_TYPE:
            *val = MODBRICK_BOARD_TYPE;
            break;
        default:
            return -EINVAL;
	}
	return 0;
}



int devm_modbrick_register_board(struct device *dev, struct modbrick_dev *modbrick){
    
    struct modbrick_board_info *data;
	struct board_info *board;
	int ret;
    uint8_t sw_ver, hw_ver;
    
    
    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
    
    ret = ModBrick_System_GetInfo(modbrick, &sw_ver, &hw_ver );
    if (ret < 0)
        return -EIO;

    snprintf(data->fw_ver, 2, "%02X", sw_ver);
    snprintf(data->hw_ver, 2, "%02X", hw_ver);
    
    data->desc.properties = modbrick_board_properties;
	data->desc.num_properties = ARRAY_SIZE(modbrick_board_properties);
	data->desc.get_property = modbrick_board_get_property;

    board = devm_board_info_register(dev, &data->desc, data);
	ret = PTR_ERR_OR_ZERO(board);
	if (ret) {
		dev_err(dev, "Failed to register board info\n");
		return ret;
	}

	return 0;
    
}
