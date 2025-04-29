#include <linux/module.h>
#include <linux/crc32.h>
#include <media/cam_sensor.h>

#include "cam_actuator_core.h"
#include "cam_actuator_soc.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "oplus_cam_actuator.h"

int32_t oplus_cam_actuator_ignore_init_error(struct cam_actuator_ctrl_t *a_ctrl, int32_t rc)
{
	if(a_ctrl->is_af_ignore_init_error != 0)
	{
		return 0;
	} else {
		return rc;
	}
}
