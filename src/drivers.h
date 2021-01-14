/*
 * Copyright (c) 2014 Bastien Nocera <hadess@hadess.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation.
 */

#include <glib.h>
#include <gudev/gudev.h>

#include "accel-attributes.h"
#include "accel-scale.h"

typedef enum {
	DRIVER_TYPE_ACCEL,
	DRIVER_TYPE_LIGHT,
	DRIVER_TYPE_COMPASS,
	DRIVER_TYPE_PROXIMITY,
} DriverType;

/* Driver types */
typedef enum {
	DRIVER_TYPE_ACCEL_IIO,
	DRIVER_TYPE_ACCEL_INPUT
} DriverAccelType;

typedef enum {
	DRIVER_TYPE_LIGHT_IIO,
	DRIVER_TYPE_LIGHT_FAKE,
	DRIVER_TYPE_LIGHT_HWMON
} DriverLightType;

typedef enum {
  DRIVER_TYPE_COMPASS_IIO,
  DRIVER_TYPE_COMPASS_FAKE
} DriverTypeCompass;

typedef enum {
  DRIVER_TYPE_PROXIMITY_IIO,
} DriverTypeProximity;

typedef enum {
  PROXIMITY_NEAR_ERROR = -1,
  PROXIMITY_NEAR_FALSE =  0,
  PROXIMITY_NEAR_TRUE  =  1,
} ProximityNear;

typedef struct SensorDriver SensorDriver;

typedef struct {
	int accel_x;
	int accel_y;
	int accel_z;
	AccelScale scale;
} AccelReadings;

typedef struct {
	gdouble  level;
	gboolean uses_lux;
} LightReadings;

typedef struct {
	gdouble heading;
} CompassReadings;

typedef struct {
	ProximityNear is_near;
} ProximityReadings;

typedef void (*ReadingsUpdateFunc) (SensorDriver *driver,
				    gpointer      readings,
				    gpointer      user_data);

typedef struct SensorDevice SensorDevice;

struct SensorDriver {
	const char             *name;
	DriverType              type;

	gboolean       (*discover)    (GUdevDevice        *device);
	SensorDevice * (*open)        (GUdevDevice        *device,
				       ReadingsUpdateFunc  callback_func,
				       gpointer            user_data);
	void           (*set_polling) (SensorDevice       *device,
				       gboolean            state);
	void           (*close)       (SensorDevice       *device);
};

struct SensorDevice {
	struct SensorDriver *drv;
	gpointer priv;
};

static inline gboolean
driver_discover (SensorDriver *driver,
		 GUdevDevice  *device)
{
	g_return_val_if_fail (driver, FALSE);
	g_return_val_if_fail (driver->discover, FALSE);
	g_return_val_if_fail (device, FALSE);

	if (!driver->discover (device))
		return FALSE;

	if (driver->type != DRIVER_TYPE_ACCEL)
		return TRUE;

	return (setup_accel_location (device) == ACCEL_LOCATION_DISPLAY);
}

static inline SensorDevice *
driver_open (SensorDriver       *driver,
	     GUdevDevice        *device,
	     ReadingsUpdateFunc  callback_func,
	     gpointer            user_data)
{
	SensorDevice *sensor_device;

	g_return_val_if_fail (driver, NULL);
	g_return_val_if_fail (driver->open, NULL);
	g_return_val_if_fail (device, NULL);
	g_return_val_if_fail (callback_func, NULL);

	sensor_device = driver->open (device, callback_func, user_data);
	if (!sensor_device)
		return NULL;
	sensor_device->drv = driver;
	return sensor_device;
}

static inline void
driver_set_polling (SensorDevice *sensor_device,
		    gboolean      state)
{
	SensorDriver *driver;

	g_return_if_fail (sensor_device);
	driver = sensor_device->drv;
	g_return_if_fail (driver);

	if (!driver->set_polling)
		return;

	driver->set_polling (sensor_device, state);
}

static inline void
driver_close (SensorDevice *sensor_device)
{
	SensorDriver *driver;

	g_return_if_fail (sensor_device);
	driver_set_polling (sensor_device, FALSE);
	driver = sensor_device->drv;
	g_return_if_fail (driver->close);
	driver->close (sensor_device);
}

extern SensorDriver iio_buffer_accel;
extern SensorDriver iio_poll_accel;
extern SensorDriver input_accel;
extern SensorDriver fake_compass;
extern SensorDriver fake_light;
extern SensorDriver iio_poll_light;
extern SensorDriver hwmon_light;
extern SensorDriver iio_buffer_light;
extern SensorDriver iio_buffer_compass;
extern SensorDriver iio_poll_proximity;

gboolean drv_check_udev_sensor_type (GUdevDevice *device, const gchar *match, const char *name);
