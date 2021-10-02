/*
 * Copyright (c) 2020 Evangelos Ribeiro Tzaras <devrtz@fortysixandtwo.eu>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation.
 */

#include "drivers.h"
#include "iio-buffer-utils.h"

#include <string.h>
#include <errno.h>
#include <math.h>

typedef struct {
  gint               x_max;
  gint               x_min;
  gint               y_max;
  gint               y_min;
  gint               z_max;
  gint               z_min;
  gboolean           is_calibrated;
} CalibrationData;

typedef struct {
  guint               timeout_id;
	ReadingsUpdateFunc  callback_func;
	gpointer            user_data;

	GUdevDevice        *dev;
	const char         *dev_path;
	const char         *name;

  CalibrationData    *calibration_data;
} DrvData;

static DrvData *drv_data = NULL;

static int
sysfs_get_int (GUdevDevice *dev,
               const char  *attribute)
{
	int result;
	char *contents;
	char *filename;

	result = 0;
	filename = g_build_filename (g_udev_device_get_sysfs_path (dev), attribute, NULL);
	if (g_file_get_contents (filename, &contents, NULL, NULL)) {
		result = atoi (contents);
		g_free (contents);
	}
	g_free (filename);

	return result;
}

static gboolean
poll_heading (gpointer user_data)
{
	DrvData *data = user_data;
  int magn_x, magn_y, magn_z;
  CompassReadings readings;
  double avg_delta_x, avg_delta_y, avg_delta_z, avg_delta;
  double offset_x, offset_y, offset_z;
  double scale_x, scale_y, scale_z, corrected_x, corrected_y, corrected_z;

  magn_x = sysfs_get_int (data->dev, "in_magn_x_raw");
  magn_y = sysfs_get_int (data->dev, "in_magn_y_raw");
  magn_z = sysfs_get_int (data->dev, "in_magn_z_raw");

  if (data->calibration_data->is_calibrated)
    {
      offset_x = (data->calibration_data->x_min + data->calibration_data->x_max) / 2;
      offset_y = (data->calibration_data->y_min + data->calibration_data->y_max) / 2;
      offset_z = (data->calibration_data->z_min + data->calibration_data->z_max) / 2;

      avg_delta_x = (data->calibration_data->x_max - data->calibration_data->x_min) / 2;
      avg_delta_y = (data->calibration_data->y_max - data->calibration_data->y_min) / 2;
      avg_delta_z = (data->calibration_data->z_max - data->calibration_data->z_min) / 2;

      avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

      scale_x = avg_delta / avg_delta_x;
      scale_y = avg_delta / avg_delta_y;
      scale_z = avg_delta / avg_delta_z;

      corrected_x = (magn_x - offset_x) * scale_x;
      corrected_y = (magn_y - offset_y) * scale_y;
      //corrected_z = (magn_z - offset_z) * scale_z;

      readings.heading = atan2 (corrected_x, corrected_y) * 180 / G_PI;
    }
  else
    readings.heading = atan2 (magn_x, magn_y) * 180 / G_PI;

  // Mount matrix?

  drv_data->callback_func (&iio_poll_compass_uncalibrated, (gpointer) &readings, drv_data->user_data);

	return G_SOURCE_CONTINUE;
}

gboolean
iio_compass_discover (GUdevDevice *device)
{
	return drv_check_udev_sensor_type (device, "iio-poll-compass-uncalibrated", "IIO poll compass uncalibrated");
}

gboolean iio_compass_open (GUdevDevice *device,
                           ReadingsUpdateFunc callback_func,
                           gpointer user_data)
{
  iio_fixup_sampling_frequency (device);
	drv_data = g_new0 (DrvData, 1);
  drv_data->calibration_data = g_new0 (CalibrationData, 1);
  drv_data->calibration_data->x_min = -7912;
  drv_data->calibration_data->x_max = -1648;
  drv_data->calibration_data->y_min = -6554;
  drv_data->calibration_data->y_max = -528;
  drv_data->calibration_data->z_min = -3074;
  drv_data->calibration_data->z_max = 2720;
  drv_data->calibration_data->is_calibrated = TRUE;

	drv_data->dev = g_object_ref (device);
	drv_data->name = g_udev_device_get_sysfs_attr (device, "name");

	drv_data->callback_func = callback_func;
	drv_data->user_data = user_data;

	return TRUE;
}

void iio_compass_set_polling (gboolean state)
{
 	if (drv_data->timeout_id > 0 && state)
		return;
	if (drv_data->timeout_id == 0 && !state)
		return;

	if (drv_data->timeout_id) {
		g_source_remove (drv_data->timeout_id);
		drv_data->timeout_id = 0;
	}

	if (state) {
		drv_data->timeout_id = g_timeout_add (700, poll_heading, drv_data);
		g_source_set_name_by_id (drv_data->timeout_id, "[iio_compass_set_polling] poll_heading");
	}
}

void iio_compass_close (void)
{
 	iio_compass_set_polling (FALSE);
	g_clear_object (&drv_data->dev);
  g_free (&drv_data->calibration_data);
	g_clear_pointer (&drv_data, g_free);
}

SensorDriver iio_poll_compass_uncalibrated = {
  .name = "IIO Poll Uncalibrated Compass",
  .type = DRIVER_TYPE_COMPASS,
  .specific_type = DRIVER_TYPE_COMPASS_IIO_UNCALIBRATED,

  .discover = iio_compass_discover,
  .open = iio_compass_open,
  .set_polling = iio_compass_set_polling,
  .close = iio_compass_close,
};
