
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>


#include "modbrick.h"


#define MODBRICK_LEDS_MAX_BRIGHTNESS    ( 255U )
#define MODBRICK_LEDS_MAX_NAME_SIZE     ( 64U )

struct modbrick_leds {
    struct modbrick_dev *modbrick;
    char name[MODBRICK_LEDS_MAX_NAME_SIZE];
    struct led_classdev cdev;
};
   
 
static inline struct modbrick_leds * to_modbrick_leds(struct led_classdev *cdev){    
    return container_of(cdev, struct modbrick_leds, cdev);
}


static int modbrick_leds_brightness_set_blocking(struct led_classdev *cdev, enum led_brightness brightness){
	struct modbrick_leds *data = to_modbrick_leds(cdev);

	return ModBrick_LED_Write(data->modbrick, 0U, (uint8_t) brightness);
}

static void modbrick_leds_release(struct device *dev, void *res){ 
    struct modbrick_leds *data = res;
    
    led_classdev_unregister(&data->cdev);
    ModBrick_LED_Write(data->modbrick, 0U, (uint8_t) 0U);
}

int devm_modbrick_register_leds(struct device *dev, struct modbrick_dev *modbrick){
    struct modbrick_leds * data;
    int ret;
    
    data = devres_alloc(modbrick_leds_release, sizeof(*data), GFP_KERNEL);
    if(!data)
        return -ENOMEM;
    
    data->modbrick = modbrick;
    snprintf(data->name, MODBRICK_LEDS_MAX_NAME_SIZE, "led:green:brick-status");
    data->cdev.name = data->name;
    data->cdev.max_brightness = MODBRICK_LEDS_MAX_BRIGHTNESS;
    data->cdev.brightness_set_blocking = modbrick_leds_brightness_set_blocking;
    data->cdev.default_trigger = "heartbeat";
    
    ret = led_classdev_register(dev, &data->cdev);
    if(ret < 0){
        devres_free(data);
        return ret;
    }
    
    devres_add(dev, data);
    
    return 0;    
}