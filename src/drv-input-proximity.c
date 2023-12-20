#include "drivers.h"
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>

#include <glib/gstdio.h>


/* we must use this kernel-compatible implementation */
#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)    ((array[LONG(bit)] >> OFF(bit)) & 1)

typedef struct DrvData {
	GUdevDevice        *dev;
	gchar              *native_path;
	int                last_switch_state;
	GIOChannel         *channel;
	struct input_event event;
	gsize              offset;
	guint              watch_id;
} DrvData;

static gint
input_str_to_bitmask (const gchar *s, glong *bitmask, size_t max_size)
{
	gint i, j;
	gchar **v;
	gint num_bits_set = 0;

	memset (bitmask, 0, max_size);
	v = g_strsplit (s, " ", max_size);
	for (i = g_strv_length (v) - 1, j = 0; i >= 0; i--, j++) {
		gulong val;

		val = strtoul (v[i], NULL, 16);
	bitmask[j] = val;

		while (val != 0) {
			num_bits_set++;
		val &= (val - 1);
		}
	}
	g_strfreev(v);

	return num_bits_set;
}

static gboolean
switch_get_bitmask (GUdevDevice *device, glong *bitmask)
{
	gboolean ret = FALSE;
	gchar *contents = NULL;
	gchar *native_path = NULL;
	gchar *path;
	GError *error = NULL;
	gint num_bits;

	/* get sysfs path */
	native_path = g_strdup (g_udev_device_get_sysfs_path (device));

	/* is a switch */
	path = g_build_filename (native_path, "../capabilities/sw", NULL);
	if (!g_file_test (path, G_FILE_TEST_EXISTS)) {
		goto out;
	}

	/* get caps */
	ret = g_file_get_contents (path, &contents, NULL, &error);
	if (!ret) {
	g_error_free (error);
		goto out;
	}

	/* convert to a bitmask */
	num_bits = input_str_to_bitmask (contents, bitmask, sizeof (bitmask));
	if ((num_bits == 0) || (num_bits >= SW_CNT)) {
		ret = FALSE;
		goto out;
	}
	ret = TRUE;

out:
	g_free (native_path);
	g_free (path);
	g_free (contents);
	return ret;
}

static gboolean
proximity_changed (GIOChannel *channel, GIOCondition condition, gpointer data) {
	/* UpInput *input = (UpInput*) data; */
	GError *error = NULL;
	gsize read_bytes;
	SensorDevice *sensor_device = data;
	DrvData *drv_data = (DrvData *) sensor_device->priv;
	glong bitmask[NBITS(SW_MAX)];
	ProximityReadings readings;

	/* uninteresting */
	if (condition & (G_IO_HUP | G_IO_ERR | G_IO_NVAL))
		return FALSE;

	/* read event */
	while (g_io_channel_read_chars (channel,
		((gchar*)&drv_data->event) + drv_data->offset,
		sizeof(struct input_event) - drv_data->offset,
		&read_bytes, &error) == G_IO_STATUS_NORMAL) {

		/* not enough data */
		if (drv_data->offset + read_bytes < sizeof (struct input_event)) {
			drv_data->offset = drv_data->offset + read_bytes;
			return TRUE;
		}

		/* we have all the data */
		drv_data->offset = 0;

		/* switch? */
		if (drv_data->event.type != EV_SW) {
			continue;
		}

		/* is not SW_FRONT_PROXIMITY */
		if (drv_data->event.code != SW_FRONT_PROXIMITY) {
			continue;
		}

		/* check switch state */
		if (ioctl (g_io_channel_unix_get_fd(channel), EVIOCGSW(sizeof (bitmask)), bitmask) < 0) {
			g_debug ("ioctl EVIOCGSW failed");
			continue;
		}

		/* are we set? */
		drv_data->last_switch_state = test_bit (drv_data->event.code, bitmask);
		readings.is_near = drv_data->last_switch_state ? PROXIMITY_NEAR_TRUE : PROXIMITY_NEAR_FALSE;
		sensor_device->callback_func (sensor_device, (gpointer) &readings, sensor_device->user_data);
	}
	return TRUE;
}

static gboolean
watch_input_proximity (gpointer user_data)
{
	SensorDevice *sensor_device = user_data;
	DrvData *drv_data = (DrvData *) sensor_device->priv;
	int eventfd;
	const gchar *device_file;
	GError *error = NULL;
	GIOStatus status;
	glong bitmask[NBITS(SW_MAX)];

	/* get device file */
	device_file = g_udev_device_get_device_file (drv_data->dev);
	if (device_file == NULL || device_file[0] == '\0') {
		return FALSE;
	}

	/* open device file */
	eventfd = open (device_file, O_RDONLY | O_NONBLOCK);
	if (eventfd < 0) {
		return FALSE;
	}

	/* get initial state */
	if (ioctl (eventfd, EVIOCGSW(sizeof (bitmask)), bitmask) < 0) {
		g_warning ("ioctl EVIOCGSW on %s failed", drv_data->native_path);
		close(eventfd);
		return FALSE;
	}

	/* create channel */
	g_debug ("watching %s (%i)", device_file, eventfd);
	drv_data->channel = g_io_channel_unix_new (eventfd);
	g_io_channel_set_close_on_unref (drv_data->channel, TRUE);

	/* set binary encoding */
	status = g_io_channel_set_encoding (drv_data->channel, NULL, &error);
	if (status != G_IO_STATUS_NORMAL) {
		g_error_free (error);
		return FALSE;
	}

	/* watch this */
	drv_data->watch_id = g_io_add_watch (drv_data->channel, G_IO_IN | G_IO_ERR | G_IO_HUP | G_IO_NVAL, proximity_changed, sensor_device);

	drv_data->last_switch_state = test_bit (SW_FRONT_PROXIMITY, bitmask);
	return TRUE;
}

static gboolean
input_proximity_discover (GUdevDevice *device)
{
	glong bitmask[NBITS(SW_MAX)];
	gboolean ret = FALSE;

	if (!switch_get_bitmask (device, bitmask))
		return ret;

	/* is this SW_FRONT_PROXIMITY? */
	if (!test_bit (SW_FRONT_PROXIMITY, bitmask))
		return ret;

	/* Input proximity sensor found */
	g_debug ("Found input proximity sensor at %s",
		g_udev_device_get_sysfs_path (device));
	return TRUE;
}

static void
input_proximity_set_polling (SensorDevice *sensor_device, gboolean state)
{
	DrvData *drv_data = (DrvData *) sensor_device->priv;

	if (drv_data->watch_id > 0 && state)
		return;
	if (drv_data->watch_id == 0 && !state)
		return;

	g_clear_handle_id (&drv_data->watch_id, g_source_remove);
	if (state) {
		/* start watching for proximity events */
		watch_input_proximity (sensor_device);
	}
}

static SensorDevice *
input_proximity_open (GUdevDevice *device)
{
	SensorDevice *sensor_device;
	DrvData *drv_data;

	sensor_device = g_new0 (SensorDevice, 1);
	sensor_device->name = g_strdup (g_udev_device_get_name (device));
	sensor_device->priv = g_new0 (DrvData, 1);
	drv_data = (DrvData *) sensor_device->priv;

	drv_data->dev = g_object_ref (device);
	drv_data->native_path = g_strdup (g_udev_device_get_sysfs_path (device));

	return sensor_device;
}

static void
input_proximity_close (SensorDevice *sensor_device)
{
	DrvData *drv_data = (DrvData *) sensor_device->priv;

	g_clear_object (&drv_data->dev);
	g_clear_pointer (&sensor_device->priv, g_free);
	g_free (sensor_device);
}

SensorDriver input_proximity = {
	.driver_name = "Input proximity",
	.type = DRIVER_TYPE_PROXIMITY,
	.discover = input_proximity_discover,
	.open = input_proximity_open,
	.close = input_proximity_close,
	.set_polling = input_proximity_set_polling,
};
