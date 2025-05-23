// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_req_mgr_debug.h"
#include "cam_mem_mgr_api.h"

#define MAX_SESS_INFO_LINE_BUFF_LEN 256
#define MAX_RAW_BUFF_LEN  8192
#define MAX_LINE_BUFF_LEN 100

static char sess_info_buffer[MAX_SESS_INFO_LINE_BUFF_LEN];
static int cam_debug_mgr_delay_detect;
static char boot_stats_info[MAX_RAW_BUFF_LEN];
static unsigned long cam_driver_bind_latency_in_usec;
static LIST_HEAD(cam_bind_latency_list);

static int cam_req_mgr_debug_set_bubble_recovery(void *data, u64 val)
{
	struct cam_req_mgr_core_device  *core_dev = data;
	struct cam_req_mgr_core_session *session;
	int rc = 0;

	mutex_lock(&core_dev->crm_lock);

	if (!list_empty(&core_dev->session_head)) {
		list_for_each_entry(session,
			&core_dev->session_head, entry) {
			session->force_err_recovery = val;
		}
	}

	mutex_unlock(&core_dev->crm_lock);

	return rc;
}

static int cam_req_mgr_debug_get_bubble_recovery(void *data, u64 *val)
{
	struct cam_req_mgr_core_device *core_dev = data;
	struct cam_req_mgr_core_session *session;

	mutex_lock(&core_dev->crm_lock);

	if (!list_empty(&core_dev->session_head)) {
		session = list_first_entry(&core_dev->session_head,
			struct cam_req_mgr_core_session,
			entry);
		*val = session->force_err_recovery;
	}
	mutex_unlock(&core_dev->crm_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bubble_recovery, cam_req_mgr_debug_get_bubble_recovery,
	cam_req_mgr_debug_set_bubble_recovery, "%lld\n");

static int session_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t session_info_read(struct file *t_file, char *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	char *out_buffer = sess_info_buffer;
	char line_buffer[MAX_SESS_INFO_LINE_BUFF_LEN] = {0};
	struct cam_req_mgr_core_device *core_dev =
		(struct cam_req_mgr_core_device *) t_file->private_data;
	struct cam_req_mgr_core_session *session;

	memset(out_buffer, 0, MAX_SESS_INFO_LINE_BUFF_LEN);

	mutex_lock(&core_dev->crm_lock);

	if (!list_empty(&core_dev->session_head)) {
		list_for_each_entry(session,
			&core_dev->session_head, entry) {
			snprintf(line_buffer, sizeof(line_buffer),
				"session_hdl = %x \t"
				"num_links = %d\n",
				session->session_hdl, session->num_links);
			strlcat(out_buffer, line_buffer,
				sizeof(sess_info_buffer));
			for (i = 0; i < session->num_links; i++) {
				snprintf(line_buffer, sizeof(line_buffer),
					"link_hdl[%d] = 0x%x, num_devs connected = %d\n",
					i, session->links[i]->link_hdl,
					session->links[i]->num_devs);
				strlcat(out_buffer, line_buffer,
					sizeof(sess_info_buffer));
			}
		}
	}

	mutex_unlock(&core_dev->crm_lock);

	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static ssize_t session_info_write(struct file *t_file,
	const char *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	memset(sess_info_buffer, 0, MAX_SESS_INFO_LINE_BUFF_LEN);

	return 0;
}

static const struct file_operations session_info = {
	.open = session_info_open,
	.read = session_info_read,
	.write = session_info_write,
};

static ssize_t cam_boot_info_read(struct file *t_file, char *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	char *out_buffer = boot_stats_info;
	struct camera_submodule_bind_time_node *node;
	int write_len = 0;

	memset(out_buffer, 0, MAX_RAW_BUFF_LEN);

	write_len = scnprintf(out_buffer, MAX_RAW_BUFF_LEN,
		"\nOverall Camera Driver Bind Latency: %lu usec\n\n"
		"---Individual Driver Bind Latency Break-up---\n",
		cam_driver_bind_latency_in_usec);
	if (!list_empty(&cam_bind_latency_list)) {
		list_for_each_entry(node,
			&cam_bind_latency_list, list) {
			write_len += scnprintf((out_buffer + write_len),
				MAX_RAW_BUFF_LEN - write_len,
				"driver_instance = %s \t"
				"bind latency = %lu usec\n",
				node->name, node->bind_time_usec);
		}
	}

	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static const struct file_operations cam_boot_debug_info = {
	.read = cam_boot_info_read,
};

static struct dentry *debugfs_root;
int cam_req_mgr_debug_register(struct cam_req_mgr_core_device *core_dev)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	if (!cam_debugfs_available())
		return 0;

	rc = cam_debugfs_create_subdir("req_mgr", &dbgfileptr);
	if (rc) {
		CAM_ERR(CAM_MEM,"DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	debugfs_root = dbgfileptr;

	debugfs_create_file("sessions_info", 0644, debugfs_root,
		core_dev, &session_info);
	debugfs_create_file("bubble_recovery", 0644,
		debugfs_root, core_dev, &bubble_recovery);
	debugfs_create_bool("recovery_on_apply_fail", 0644,
		debugfs_root, &core_dev->recovery_on_apply_fail);
	debugfs_create_bool("disable_sensor_standby", 0644,
		debugfs_root, &core_dev->disable_sensor_standby);
	debugfs_create_u32("delay_detect_count", 0644, debugfs_root,
		&cam_debug_mgr_delay_detect);
	debugfs_create_file("cam_print_boot_stats", 0644, debugfs_root,
		NULL, &cam_boot_debug_info);
end:
	return rc;
}

int cam_req_mgr_debug_unregister(void)
{
	debugfs_root = NULL;
	return 0;
}

void cam_req_mgr_debug_delay_detect(void)
{
	cam_debug_mgr_delay_detect += 1;
}

void cam_req_mgr_debug_record_bind_time(unsigned long time_in_usec)
{
	cam_driver_bind_latency_in_usec = time_in_usec;
}

void cam_req_mgr_debug_bind_latency_cleanup(void)
{
	struct camera_submodule_bind_time_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &cam_bind_latency_list, list) {
		list_del(&node->list);
		kfree(node->name);
		CAM_MEM_FREE(node);
	}
}

void cam_req_mgr_debug_record_bind_latency(const char *driver_name, unsigned long time_in_usec)
{
	struct camera_submodule_bind_time_node  *new_node =
		CAM_MEM_ZALLOC(sizeof(struct camera_submodule_bind_time_node), GFP_KERNEL);

	if (!new_node) {
		CAM_WARN(CAM_REQ, "%s: %u usec: Failed to allocate Bind Time node",
			driver_name, time_in_usec);
		return;
	}
	CAM_DBG(CAM_REQ, "%s: bind latency: %u", driver_name, time_in_usec);
	new_node->name = kstrdup(driver_name, GFP_KERNEL);
	if (!new_node->name) {
		CAM_WARN(CAM_REQ, "%s: %u usec: Failed to create driver_name",
			driver_name, time_in_usec);
		CAM_MEM_FREE(new_node);
		return;
	}
	new_node->bind_time_usec = time_in_usec;
	list_add_tail(&new_node->list, &cam_bind_latency_list);
}

