/*
 * Copyright (c) 2012-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC:  wlan_hdd_cfg.c
 *
 * WLAN Host Device Driver configuration interface implementation
 */

/* Include Files */

#include <linux/firmware.h>
#include <linux/string.h>
#include <wlan_hdd_includes.h>
#include <wlan_hdd_main.h>
#include <wlan_hdd_assoc.h>
#include <wlan_hdd_cfg.h>
#include <linux/string.h>
#include <qdf_types.h>
#include <csr_api.h>
#include <wlan_hdd_misc.h>
#include <wlan_hdd_napi.h>
#include <cds_api.h>
#include <wlan_hdd_regulatory.h>
#include "wlan_hdd_he.h"
#include <wlan_policy_mgr_api.h>
#include "wifi_pos_api.h"
#include "wlan_hdd_green_ap.h"
#include "wlan_hdd_twt.h"
#include "wlan_policy_mgr_ucfg.h"
#include "wlan_mlme_ucfg_api.h"
#include "wlan_mlme_public_struct.h"
#include "wlan_fwol_ucfg_api.h"
#include "cfg_ucfg_api.h"
#include "hdd_dp_cfg.h"
#include <wma_api.h>
#include "wlan_hdd_object_manager.h"
#include "wlan_dp_ucfg_api.h"
#include "wlan_cmn.h"

#ifndef WLAN_MAC_ADDR_UPDATE_DISABLE
/**
 * get_next_line() - find and locate the new line pointer
 * @str: pointer to string
 *
 * This function returns a pointer to the character after the occurrence
 * of a new line character. It also modifies the original string by replacing
 * the '\n' character with the null character.
 *
 * Return: the pointer to the character at new line,
 *            or NULL if no new line character was found
 */
static char *get_next_line(char *str)
{
	char c;

	if (!str || *str == '\0')
		return NULL;

	c = *str;
	while (c != '\n' && c != '\0' && c != 0xd) {
		str = str + 1;
		c = *str;
	}

	if (c == '\0')
		return NULL;

	*str = '\0';
	return str + 1;
}

/** look for space. Ascii values to look are
 * 0x09 == horizontal tab
 * 0x0a == Newline ("\n")
 * 0x0b == vertical tab
 * 0x0c == Newpage or feed form.
 * 0x0d == carriage return (CR or "\r")
 * Null ('\0') should not considered as space.
 */
#define i_isspace(ch)  (((ch) >= 0x09 && (ch) <= 0x0d) || (ch) == ' ')

/**
 * i_trim() - trims any leading and trailing white spaces
 * @str: pointer to string
 *
 * Return: the pointer of the string
 */
static char *i_trim(char *str)
{
	char *ptr;

	if (*str == '\0')
		return str;

	/* Find the first non white-space */
	ptr = str;
	while (i_isspace(*ptr))
		ptr++;

	if (*ptr == '\0')
		return str;

	/* This is the new start of the string */
	str = ptr;

	/* Find the last non white-space */
	ptr += strlen(ptr) - 1;

	while (ptr != str && i_isspace(*ptr))
		ptr--;

	/* Null terminate the following character */
	ptr[1] = '\0';

	return str;
}

/** struct hdd_cfg_entry - ini configuration entry
 * @name: name of the entry
 * @value: value of the entry
 */
struct hdd_cfg_entry {
	char *name;
	char *value;
};

/**
 * update_mac_from_string() - convert string to 6 bytes mac address
 * @hdd_ctx: the pointer to hdd context
 * @mac_table: the mac_table to carry the conversion
 * @num: number of the interface
 *
 * 00AA00BB00CC -> 0x00 0xAA 0x00 0xBB 0x00 0xCC
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS update_mac_from_string(struct hdd_context *hdd_ctx,
					 struct hdd_cfg_entry *mac_table,
					 int num)
{
	int i = 0, j = 0, res = 0;
	char *candidate = NULL;
	struct qdf_mac_addr macaddr[QDF_MAX_CONCURRENCY_PERSONA];
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	memset(macaddr, 0, sizeof(macaddr));

	for (i = 0; i < num; i++) {
		candidate = mac_table[i].value;
		for (j = 0; j < QDF_MAC_ADDR_SIZE; j++) {
			res =
				hex2bin(&macaddr[i].bytes[j], &candidate[(j << 1)],
					1);
			if (res < 0)
				break;
		}
		if (res == 0 && !qdf_is_macaddr_zero(&macaddr[i])) {
			qdf_mem_copy((uint8_t *)&hdd_ctx->
				     provisioned_mac_addr[i].bytes[0],
				     (uint8_t *) &macaddr[i].bytes[0],
				     QDF_MAC_ADDR_SIZE);
		} else {
			status = QDF_STATUS_E_FAILURE;
			break;
		}
	}
	return status;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0))
static inline
int hdd_firmware_request_nowarn(const struct firmware **fw,
				const char *name,
				struct device *device)
{
	return firmware_request_nowarn(fw, name, device);
}
#else
static inline
int hdd_firmware_request_nowarn(const struct firmware **fw,
				const char *name,
				struct device *device)
{
	return request_firmware(fw, name, device);
}
#endif

/**
 * hdd_update_mac_config() - update MAC address from cfg file
 * @hdd_ctx: the pointer to hdd context
 *
 * It overwrites the MAC address if config file exist.
 *
 * Return: QDF_STATUS_SUCCESS if the MAC address is found from cfg file
 *      and overwritten, otherwise QDF_STATUS_E_INVAL
 */
QDF_STATUS hdd_update_mac_config(struct hdd_context *hdd_ctx)
{
	int status, i = 0;
	const struct firmware *fw = NULL;
	char *line, *buffer = NULL;
	char *temp = NULL;
	char *name, *value;
	int max_mac_addr = QDF_MAX_CONCURRENCY_PERSONA;
	struct hdd_cfg_entry mac_table[QDF_MAX_CONCURRENCY_PERSONA];
	struct qdf_mac_addr custom_mac_addr;

	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;

	if (!hdd_ctx->config->read_mac_addr_from_mac_file) {
		hdd_debug("Reading MAC address from MAC file is not enabled.");
		return QDF_STATUS_E_FAILURE;
	}

	memset(mac_table, 0, sizeof(mac_table));
	status = hdd_firmware_request_nowarn(&fw, WLAN_MAC_FILE,
					     hdd_ctx->parent_dev);
	if (status) {
		/*
		 * request_firmware "fails" if the file is not found, which is a
		 * valid setup for us, so log using debug instead of error
		 */
		hdd_debug("request_firmware failed; status:%d", status);
		return QDF_STATUS_E_FAILURE;
	}

	if (!fw || !fw->data || !fw->size) {
		hdd_alert("invalid firmware");
		qdf_status = QDF_STATUS_E_INVAL;
		goto config_exit;
	}

	hdd_debug("wlan_mac.bin size %zu", fw->size);

	temp = qdf_mem_malloc(fw->size + 1);
	if (!temp) {
		qdf_status = QDF_STATUS_E_NOMEM;
		goto config_exit;
	}
	buffer = temp;
	qdf_mem_copy(buffer, fw->data, fw->size);
	buffer[fw->size] = 0x0;

	/* data format:
	 * Intf0MacAddress=00AA00BB00CC
	 * Intf1MacAddress=00AA00BB00CD
	 * END
	 */
	while (buffer) {
		line = get_next_line(buffer);
		buffer = i_trim(buffer);

		if (strlen((char *)buffer) == 0 || *buffer == '#') {
			buffer = line;
			continue;
		}
		if (strncmp(buffer, "END", 3) == 0)
			break;

		name = buffer;
		buffer = strnchr(buffer, strlen(buffer), '=');
		if (buffer) {
			*buffer++ = '\0';
			i_trim(name);
			if (strlen(name) != 0) {
				buffer = i_trim(buffer);
				if (strlen(buffer) == 12) {
					value = buffer;
					mac_table[i].name = name;
					mac_table[i++].value = value;
					if (i >= QDF_MAX_CONCURRENCY_PERSONA)
						break;
				}
			}
		}
		buffer = line;
	}

	if (i != 0 && i <= QDF_MAX_CONCURRENCY_PERSONA) {
		hdd_debug("%d Mac addresses provided", i);
	} else {
		hdd_err("invalid number of Mac address provided, nMac = %d", i);
		qdf_status = QDF_STATUS_E_INVAL;
		goto config_exit;
	}

	qdf_status = update_mac_from_string(hdd_ctx, &mac_table[0], i);
	if (QDF_IS_STATUS_ERROR(qdf_status)) {
		hdd_err("Invalid MAC addresses provided");
		goto config_exit;
	}
	hdd_ctx->num_provisioned_addr = i;
	hdd_debug("Populating remaining %d Mac addresses",
		   max_mac_addr - i);
	hdd_populate_random_mac_addr(hdd_ctx, max_mac_addr - i);

	if (hdd_ctx->num_provisioned_addr)
		qdf_mem_copy(custom_mac_addr.bytes,
			     &hdd_ctx->provisioned_mac_addr[0].bytes[0],
			     sizeof(custom_mac_addr));
	else
		qdf_mem_copy(custom_mac_addr.bytes,
			     &hdd_ctx->derived_mac_addr[0].bytes[0],
			     sizeof(custom_mac_addr));

	qdf_status = sme_set_custom_mac_addr(custom_mac_addr.bytes);

config_exit:
	qdf_mem_free(temp);
	release_firmware(fw);
	return qdf_status;
}
#else
QDF_STATUS hdd_update_mac_config(struct hdd_context *hdd_ctx)
{
	return QDF_STATUS_E_NOSUPPORT;
}
#endif

/**
 * hdd_set_power_save_offload_config() - set power save offload configuration
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: none
 */
static void hdd_set_power_save_offload_config(struct hdd_context *hdd_ctx)
{
	uint32_t listen_interval = 0;
	char *power_usage = NULL;

	power_usage = ucfg_mlme_get_power_usage(hdd_ctx->psoc);
	if (!power_usage) {
		hdd_err("invalid power usage");
		return;
	}

	if (strcmp(power_usage, "Min") == 0)
		ucfg_mlme_get_bmps_min_listen_interval(hdd_ctx->psoc,
						       &listen_interval);
	else if (strcmp(power_usage, "Max") == 0)
		ucfg_mlme_get_bmps_max_listen_interval(hdd_ctx->psoc,
						       &listen_interval);
	/*
	 * Based on Mode Set the LI
	 * Otherwise default LI value of 1 will
	 * be taken
	 */
	if (listen_interval) {
		/*
		 * setcfg for listenInterval.
		 * Make sure CFG is updated because PE reads this
		 * from CFG at the time of assoc or reassoc
		 */
		ucfg_mlme_set_sap_listen_interval(hdd_ctx->psoc,
						  listen_interval);
	}
}

#ifdef FEATURE_RUNTIME_PM
/**
 * hdd_disable_runtime_pm() - Override to disable runtime_pm.
 * @cfg_ini: Handle to struct hdd_config
 *
 * Return: None
 */
static void hdd_disable_runtime_pm(struct hdd_config *cfg_ini)
{
	cfg_ini->runtime_pm = 0;
}

/**
 * hdd_restore_runtime_pm() - Restore runtime_pm configuration.
 * @hdd_ctx: HDD context
 *
 * Return: None
 */
static void hdd_restore_runtime_pm(struct hdd_context *hdd_ctx)
{
	struct hdd_config *cfg_ini = hdd_ctx->config;

	cfg_ini->runtime_pm = cfg_get(hdd_ctx->psoc, CFG_ENABLE_RUNTIME_PM);
}
#else
static void hdd_disable_runtime_pm(struct hdd_config *cfg_ini)
{
}

static void hdd_restore_runtime_pm(struct hdd_context *hdd_ctx)
{
}
#endif

#ifdef FEATURE_WLAN_AUTO_SHUTDOWN
/**
 * hdd_disable_auto_shutdown() - Override to disable auto_shutdown.
 * @cfg_ini: Handle to struct hdd_config
 *
 * Return: None
 */
static void hdd_disable_auto_shutdown(struct hdd_config *cfg_ini)
{
	cfg_ini->wlan_auto_shutdown = 0;
}

/**
 * hdd_restore_auto_shutdown() - Restore auto_shutdown configuration.
 * @hdd_ctx: HDD context
 *
 * Return: None
 */
static void hdd_restore_auto_shutdown(struct hdd_context *hdd_ctx)
{
	struct hdd_config *cfg_ini = hdd_ctx->config;

	cfg_ini->wlan_auto_shutdown = cfg_get(hdd_ctx->psoc,
					      CFG_WLAN_AUTO_SHUTDOWN);
}
#else
static void hdd_disable_auto_shutdown(struct hdd_config *cfg_ini)
{
}

static void hdd_restore_auto_shutdown(struct hdd_context *hdd_ctx)
{
}
#endif

void hdd_restore_all_ps(struct hdd_context *hdd_ctx)
{
	/*
	 * imps/bmps configuration will be restored in driver mode change
	 * sequence as part of hdd_wlan_start_modules
	 */

	hdd_restore_runtime_pm(hdd_ctx);
	hdd_restore_auto_shutdown(hdd_ctx);
}

void hdd_override_all_ps(struct hdd_context *hdd_ctx)
{
	struct hdd_config *cfg_ini = hdd_ctx->config;

	ucfg_mlme_override_bmps_imps(hdd_ctx->psoc);
	hdd_disable_runtime_pm(cfg_ini);
	hdd_disable_auto_shutdown(cfg_ini);
}

/**
 * hdd_cfg_xlate_to_csr_phy_mode() - convert PHY mode
 * @dot11Mode: the mode to convert
 *
 * Convert the configuration PHY mode to CSR PHY mode
 *
 * Return: the CSR phy mode value
 */
eCsrPhyMode hdd_cfg_xlate_to_csr_phy_mode(enum hdd_dot11_mode dot11Mode)
{
	if (cds_is_sub_20_mhz_enabled())
		return eCSR_DOT11_MODE_abg;

	switch (dot11Mode) {
	case (eHDD_DOT11_MODE_abg):
		return eCSR_DOT11_MODE_abg;
	case (eHDD_DOT11_MODE_11b):
		return eCSR_DOT11_MODE_11b;
	case (eHDD_DOT11_MODE_11g):
		return eCSR_DOT11_MODE_11g;
	default:
	case (eHDD_DOT11_MODE_11n):
		return eCSR_DOT11_MODE_11n;
	case (eHDD_DOT11_MODE_11g_ONLY):
		return eCSR_DOT11_MODE_11g_ONLY;
	case (eHDD_DOT11_MODE_11n_ONLY):
		return eCSR_DOT11_MODE_11n_ONLY;
	case (eHDD_DOT11_MODE_11b_ONLY):
		return eCSR_DOT11_MODE_11b_ONLY;
	case (eHDD_DOT11_MODE_11ac_ONLY):
		return eCSR_DOT11_MODE_11ac_ONLY;
	case (eHDD_DOT11_MODE_11ac):
		return eCSR_DOT11_MODE_11ac;
	case (eHDD_DOT11_MODE_AUTO):
		return eCSR_DOT11_MODE_AUTO;
	case (eHDD_DOT11_MODE_11a):
		return eCSR_DOT11_MODE_11a;
	case (eHDD_DOT11_MODE_11ax_ONLY):
		return eCSR_DOT11_MODE_11ax_ONLY;
	case (eHDD_DOT11_MODE_11ax):
		return eCSR_DOT11_MODE_11ax;
#ifdef WLAN_FEATURE_11BE
	case (eHDD_DOT11_MODE_11be):
		return eCSR_DOT11_MODE_11be;
	case (eHDD_DOT11_MODE_11be_ONLY):
		return eCSR_DOT11_MODE_11be_ONLY;
#endif
	}

}

/**
 * hdd_set_idle_ps_config() - set idle power save configuration
 * @hdd_ctx: the pointer to hdd context
 * @val: the value to configure
 *
 * Return: QDF_STATUS_SUCCESS if command set correctly,
 *		otherwise the QDF_STATUS return from SME layer
 */
QDF_STATUS hdd_set_idle_ps_config(struct hdd_context *hdd_ctx, bool val)
{
	QDF_STATUS status;

	hdd_debug("Enter Val %d", val);

	if (hdd_get_conparam() == QDF_GLOBAL_FTM_MODE) {
		hdd_debug("Skipping powersave in FTM");
		return QDF_STATUS_SUCCESS;
	}

	if (hdd_ctx->imps_enabled == val) {
		hdd_nofl_debug("Already in the requested power state:%d", val);
		return QDF_STATUS_SUCCESS;
	}

	status = sme_set_idle_powersave_config(val);
	if (QDF_STATUS_SUCCESS != status) {
		hdd_err("Fail to Set Idle PS Config val %d", val);
		return status;
	}

	hdd_ctx->imps_enabled = val;

	return status;
}

/**
 * hdd_set_fine_time_meas_cap() - set fine timing measurement capability
 * @hdd_ctx: HDD context
 *
 * This function is used to pass fine timing measurement capability coming
 * from INI to SME. This function make sure that configure INI is supported
 * by the device. Use bit mask to mask out the unsupported capabilities.
 *
 * Return: None
 */
static void hdd_set_fine_time_meas_cap(struct hdd_context *hdd_ctx)
{
	uint32_t capability = 0;

	ucfg_mlme_get_fine_time_meas_cap(hdd_ctx->psoc, &capability);
	ucfg_wifi_pos_set_ftm_cap(hdd_ctx->psoc, capability);
	hdd_debug("fine time meas capability - Enabled: %04x", capability);
}

/**
 * hdd_set_oem_6g_supported() - set oem 6g support enabled/disable
 * @hdd_ctx: HDD context
 *
 * This function is used to pass oem 6g support enabled/disable value
 * coming from INI to SME. This function make sure that configure
 * INI is supported by the device.
 *
 * Return: None
 */
static void hdd_set_oem_6g_supported(struct hdd_context *hdd_ctx)
{
	bool oem_6g_disable = true;
	bool is_reg_6g_support, set_wifi_pos_6g_disabled;

	ucfg_mlme_get_oem_6g_supported(hdd_ctx->psoc, &oem_6g_disable);
	is_reg_6g_support = wlan_reg_is_6ghz_supported(hdd_ctx->psoc);
	set_wifi_pos_6g_disabled = (oem_6g_disable || !is_reg_6g_support);

	/**
	 * Host uses following truth table to set wifi pos 6Ghz disable in
	 * ucfg_wifi_pos_set_oem_6g_supported().
	 * -----------------------------------------------------------------
	 * oem_6g_disable INI value | reg domain 6G support | Disable 6Ghz |
	 * -----------------------------------------------------------------
	 *            1             |           1           |        1     |
	 *            1             |           0           |        1     |
	 *            0             |           1           |        0     |
	 *            0             |           0           |        1     |
	 * -----------------------------------------------------------------
	 */
	ucfg_wifi_pos_set_oem_6g_supported(hdd_ctx->psoc,
					   set_wifi_pos_6g_disabled);
	hdd_debug("oem 6g support is - %s",
		  set_wifi_pos_6g_disabled ? "Disabled" : "Enabled");
}

/**
 * hdd_convert_string_to_array() - used to convert string into u8 array
 * @str: String to be converted
 * @array: Array where converted value is stored
 * @len: Length of the populated array
 * @array_max_len: Maximum length of the array
 * @to_hex: true, if conversion required for hex string
 *
 * This API is called to convert string (each byte separated by
 * a comma) into an u8 array
 *
 * Return: QDF_STATUS
 */

static QDF_STATUS hdd_convert_string_to_array(char *str, uint8_t *array,
			     uint8_t *len, uint16_t array_max_len, bool to_hex)
{
	char *format, *s = str;

	if (!str || !array || !len)
		return QDF_STATUS_E_INVAL;

	format = (to_hex) ? "%02x" : "%d";

	*len = 0;
	while ((s) && (*len < array_max_len)) {
		int val;
		/* Increment length only if sscanf successfully extracted
		 * one element. Any other return value means error.
		 * Ignore it.
		 */
		if (sscanf(s, format, &val) == 1) {
			array[*len] = (uint8_t) val;
			*len += 1;
		}

		s = strpbrk(s, ",");
		if (s)
			s++;
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS hdd_string_to_u8_array(char *str, uint8_t *array,
				  uint8_t *len, uint16_t array_max_len)
{
	return hdd_convert_string_to_array(str, array, len,
					   array_max_len, false);
}

/**
 * hdd_hex_string_to_u16_array() - convert a hex string to a uint16 array
 * @str: input string
 * @int_array: pointer to input array of type uint16
 * @len: pointer to number of elements which the function adds to the array
 * @int_array_max_len: maximum number of elements in input uint16 array
 *
 * This function is used to convert a space separated hex string to an array of
 * uint16_t. For example, an input string str = "a b c d" would be converted to
 * a unint16 array, int_array = {0xa, 0xb, 0xc, 0xd}, *len = 4.
 * This assumes that input value int_array_max_len >= 4.
 *
 * Return: QDF_STATUS_SUCCESS - if the conversion is successful
 *         non zero value     - if the conversion is a failure
 */
QDF_STATUS hdd_hex_string_to_u16_array(char *str,
		uint16_t *int_array, uint8_t *len, uint8_t int_array_max_len)
{
	char *s = str;
	uint32_t val = 0;

	if (!str || !int_array || !len)
		return QDF_STATUS_E_INVAL;

	hdd_debug("str %pK intArray %pK intArrayMaxLen %d",
		s, int_array, int_array_max_len);

	*len = 0;

	while ((s) && (*len < int_array_max_len)) {
		/*
		 * Increment length only if sscanf successfully extracted one
		 * element. Any other return value means error. Ignore it.
		 */
		if (sscanf(s, "%x", &val) == 1) {
			int_array[*len] = (uint16_t) val;
			hdd_debug("s %pK val %x intArray[%d]=0x%x",
				s, val, *len, int_array[*len]);
			*len += 1;
		}
		s = strpbrk(s, " ");
		if (s)
			s++;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * hdd_update_config_cfg() - API to update INI setting based on hw/fw caps
 * @hdd_ctx: pointer to hdd_ctx
 *
 * This API reads the cfg file which is updated with hardware/firmware
 * capabilities and intersect it with INI setting provided by user. After
 * taking intersection it adjust cfg it self. For example, if user has enabled
 * RX LDPC through INI but hardware/firmware doesn't support it then disable
 * it in CFG file here.
 *
 * Return: true or false based on outcome.
 */
bool hdd_update_config_cfg(struct hdd_context *hdd_ctx)
{
	bool status = true;

	/*
	 * During the initialization both 2G and 5G capabilities should be same.
	 * So read 5G HT capability and update 2G and 5G capabilities.
	 */

	if (0 != hdd_update_he_cap_in_cfg(hdd_ctx)) {
		status = false;
		hdd_err("Couldn't set HE CAP in cfg");
	}

	return status;
}

/**
 * hdd_set_policy_mgr_user_cfg() -initializes the policy manager
 * configuration parameters
 *
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: QDF_STATUS_SUCCESS if configuration is correctly applied,
 *		otherwise the appropriate QDF_STATUS would be returned
 */
QDF_STATUS hdd_set_policy_mgr_user_cfg(struct hdd_context *hdd_ctx)
{
	QDF_STATUS status;
	struct policy_mgr_user_cfg *user_cfg;

	user_cfg = qdf_mem_malloc(sizeof(*user_cfg));
	if (!user_cfg)
		return QDF_STATUS_E_NOMEM;

	status = ucfg_mlme_get_vht_enable2x2(hdd_ctx->psoc,
					     &user_cfg->enable2x2);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("unable to get vht_enable2x2");

	user_cfg->sub_20_mhz_enabled = cds_is_sub_20_mhz_enabled();
	status = policy_mgr_set_user_cfg(hdd_ctx->psoc, user_cfg);
	qdf_mem_free(user_cfg);

	return status;
}

enum wmm_user_mode hdd_to_csr_wmm_mode(uint8_t mode)
{
	switch (mode) {
	case HDD_WMM_USER_MODE_QBSS_ONLY:
		return WMM_USER_MODE_QBSS_ONLY;
	case HDD_WMM_USER_MODE_NO_QOS:
		return WMM_USER_MODE_NO_QOS;
	case HDD_WMM_USER_MODE_AUTO:
	default:
		return WMM_USER_MODE_AUTO;
	}
}

static QDF_STATUS
hdd_set_sme_cfgs_related_to_plcy_mgr(struct hdd_context *hdd_ctx,
				     struct sme_config_params *sme_cfg)
{
	uint8_t mcc_to_scc_switch = 0, is_force_1x1 = 0, allow_diff_bi = 0;
	uint8_t conc_rule1 = 0, conc_rule2 = 0, sta_cxn_5g = 0;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_mcc_scc_switch(hdd_ctx->psoc,
					       &mcc_to_scc_switch)) {
		hdd_err("can't get mcc to scc switch");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.cc_switch_mode = mcc_to_scc_switch;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_conc_rule1(hdd_ctx->psoc,
					   &conc_rule1)) {
		hdd_err("can't get conc rule1");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.conc_custom_rule1 = conc_rule1;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_conc_rule2(hdd_ctx->psoc,
					   &conc_rule2)) {
		hdd_err("can't get conc rule2");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.conc_custom_rule2 = conc_rule2;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_sta_cxn_5g_band(hdd_ctx->psoc,
						&sta_cxn_5g)) {
		hdd_err("can't get conc rule2");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.is_sta_connection_in_5gz_enabled = sta_cxn_5g;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_force_1x1(hdd_ctx->psoc,
					  &is_force_1x1)) {
		hdd_err("can't get force 1x1 flag");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.is_force_1x1 = is_force_1x1;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_allow_mcc_go_diff_bi(hdd_ctx->psoc,
						     &allow_diff_bi)) {
		hdd_err("can't get allow mcc go diff BI flag");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.fAllowMCCGODiffBI = allow_diff_bi;

	return QDF_STATUS_SUCCESS;
}

#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
static QDF_STATUS hdd_set_sap_mcc_chnl_avoid(struct sme_config_params *sme_cfg,
					     uint8_t val)
{
	sme_cfg->csr_config.sap_channel_avoidance = val;
	return QDF_STATUS_SUCCESS;
}
#else
static QDF_STATUS hdd_set_sap_mcc_chnl_avoid(struct sme_config_params *sme_cfg,
					     uint8_t val)
{
	return QDF_STATUS_SUCCESS;
}
#endif

static
QDF_STATUS hdd_set_sme_cfgs_related_to_mlme(struct hdd_context *hdd_ctx,
					    struct sme_config_params *sme_cfg)
{
	QDF_STATUS status;
	uint8_t wmm_mode = 0, enable_mcc = 0, sap_mcc_avoid = 0;
	uint8_t mcc_rts_cts = 0, mcc_bcast_prob_rsp = 0;
	uint32_t mcast_mcc_rest_time = 0;
	bool b80211e_enabled = 0;

	status = ucfg_mlme_get_80211e_is_enabled(hdd_ctx->psoc,
						 &b80211e_enabled);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("Get b80211e_enabled failed");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.Is11eSupportEnabled = b80211e_enabled;

	status = ucfg_mlme_get_wmm_mode(hdd_ctx->psoc, &wmm_mode);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("Get wmm_mode failed");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.WMMSupportMode = hdd_to_csr_wmm_mode(wmm_mode);
	hdd_debug("wmm_mode=%d 802_11e_enabled=%d", wmm_mode, b80211e_enabled);

	status = ucfg_mlme_get_mcc_feature(hdd_ctx->psoc, &enable_mcc);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_mcc_feature fail, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.fEnableMCCMode = enable_mcc;

	status = ucfg_mlme_get_mcc_rts_cts_prot(hdd_ctx->psoc, &mcc_rts_cts);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_mcc_rts_cts_prot fail, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.mcc_rts_cts_prot_enable = mcc_rts_cts;

	status = ucfg_mlme_get_mcc_bcast_prob_resp(hdd_ctx->psoc,
						   &mcc_bcast_prob_rsp);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_mcc_bcast_prob_resp fail, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.mcc_bcast_prob_resp_enable = mcc_bcast_prob_rsp;

	status = ucfg_mlme_get_sta_miracast_mcc_rest_time(hdd_ctx->psoc,
							  &mcast_mcc_rest_time);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_sta_miracast_mcc_rest_time, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.f_sta_miracast_mcc_rest_time_val =
							mcast_mcc_rest_time;
	status = ucfg_mlme_get_sap_mcc_chnl_avoid(hdd_ctx->psoc,
						  &sap_mcc_avoid);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_sap_mcc_chnl_avoid, use def");
		return QDF_STATUS_E_FAILURE;
	}
	status = hdd_set_sap_mcc_chnl_avoid(sme_cfg, sap_mcc_avoid);

	return status;
}

/**
 * hdd_set_sme_config() -initializes the sme configuration parameters
 *
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: QDF_STATUS_SUCCESS if configuration is correctly applied,
 *		otherwise the appropriate QDF_STATUS would be returned
 */
QDF_STATUS hdd_set_sme_config(struct hdd_context *hdd_ctx)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct sme_config_params *sme_config;
	mac_handle_t mac_handle = hdd_ctx->mac_handle;
	bool roam_scan_enabled;
	bool enable_dfs_scan = true;
	bool disconnect_nud;
	uint32_t channel_bonding_mode;

#ifdef FEATURE_WLAN_ESE
	bool ese_enabled;
#endif
	struct hdd_config *config = hdd_ctx->config;
	struct wlan_mlme_psoc_ext_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_ext_obj(hdd_ctx->psoc);
	if (!mlme_obj)
		return QDF_STATUS_E_INVAL;

	sme_config = qdf_mem_malloc(sizeof(*sme_config));
	if (!sme_config)
		return QDF_STATUS_E_NOMEM;

	/* Config params obtained from the registry
	 * To Do: set regulatory information here
	 */
	sme_config->csr_config.phyMode =
		hdd_cfg_xlate_to_csr_phy_mode(config->dot11Mode);

	if (config->dot11Mode == eHDD_DOT11_MODE_abg ||
	    config->dot11Mode == eHDD_DOT11_MODE_11b ||
	    config->dot11Mode == eHDD_DOT11_MODE_11g ||
	    config->dot11Mode == eHDD_DOT11_MODE_11b_ONLY ||
	    config->dot11Mode == eHDD_DOT11_MODE_11g_ONLY) {
		sme_config->csr_config.channelBondingMode24GHz = 0;
		sme_config->csr_config.channelBondingMode5GHz = 0;
	} else {
		ucfg_mlme_get_channel_bonding_24ghz(hdd_ctx->psoc,
						    &channel_bonding_mode);
		sme_config->csr_config.channelBondingMode24GHz =
			channel_bonding_mode;
		ucfg_mlme_get_channel_bonding_5ghz(hdd_ctx->psoc,
						   &channel_bonding_mode);
		sme_config->csr_config.channelBondingMode5GHz =
			channel_bonding_mode;
	}
	/* Remaining config params not obtained from registry
	 * On RF EVB beacon using channel 1.
	 */
	/* This param cannot be configured from INI */
	sme_config->csr_config.send_smps_action = true;
	sme_config->csr_config.ProprietaryRatesEnabled = 0;
	sme_config->csr_config.HeartbeatThresh50 = 40;
	ucfg_scan_cfg_get_dfs_chan_scan_allowed(hdd_ctx->psoc,
						&enable_dfs_scan);
	sme_config->csr_config.fEnableDFSChnlScan = enable_dfs_scan;
	sme_config->csr_config.Csr11dinfo.Channels.numChannels = 0;
	hdd_set_power_save_offload_config(hdd_ctx);

#ifdef FEATURE_WLAN_ESE
	ucfg_mlme_is_ese_enabled(hdd_ctx->psoc, &ese_enabled);
	if (ese_enabled)
		ucfg_mlme_set_fast_transition_enabled(hdd_ctx->psoc, true);
#endif

	ucfg_mlme_is_roam_scan_offload_enabled(hdd_ctx->psoc,
					       &roam_scan_enabled);

	if (!roam_scan_enabled) {
		/* Disable roaming in concurrency if roam scan
		 * offload is disabled
		 */
		ucfg_mlme_set_fast_roam_in_concurrency_enabled(
					hdd_ctx->psoc, false);
	}

	/* Update maximum interfaces information */
	sme_config->csr_config.max_intf_count = hdd_ctx->max_intf_count;

	hdd_set_fine_time_meas_cap(hdd_ctx);
	hdd_set_oem_6g_supported(hdd_ctx);

	cds_set_multicast_logging(hdd_ctx->config->multicast_host_fw_msgs);

	mlme_obj->cfg.lfr.rso_user_config.policy_params.dfs_mode =
		STA_ROAM_POLICY_DFS_ENABLED;
	mlme_obj->cfg.lfr.rso_user_config.policy_params.skip_unsafe_channels = 0;

	disconnect_nud = ucfg_dp_is_disconect_after_roam_fail(hdd_ctx->psoc);
	mlme_obj->cfg.lfr.disconnect_on_nud_roam_invoke_fail = disconnect_nud;

	status = hdd_set_sme_cfgs_related_to_mlme(hdd_ctx, sme_config);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("hdd_set_sme_cfgs_related_to_mlme() fail: %d", status);
	status = hdd_set_sme_cfgs_related_to_plcy_mgr(hdd_ctx, sme_config);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("hdd_set_sme_cfgs_related_to_plcy_mgr fail: %d",
			status);
	hdd_debug("dot11Mode=%d", config->dot11Mode);
	status = sme_update_config(mac_handle, sme_config);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("sme_update_config() failure: %d", status);

	qdf_mem_free(sme_config);
	return status;
}

/**
 * hdd_cfg_get_global_config() - get the configuration table
 * @hdd_ctx: pointer to hdd context
 * @buf: buffer to store the configuration
 * @buflen: size of the buffer
 *
 * Return: none
 */
void hdd_cfg_get_global_config(struct hdd_context *hdd_ctx, char *buf,
			       int buflen)
{
	ucfg_cfg_store_print(hdd_ctx->psoc);

	snprintf(buf, buflen,
		 "WLAN configuration written to debug log");
}

/**
 * hdd_cfg_print_global_config() - print the configuration table
 * @hdd_ctx: pointer to hdd context
 *
 * Return: none
 */
void hdd_cfg_print_global_config(struct hdd_context *hdd_ctx)
{
	QDF_STATUS status;

	status = ucfg_cfg_store_print(hdd_ctx->psoc);
	if (QDF_IS_STATUS_ERROR(status))
		hdd_err("Failed to log cfg ini");
}

/**
 * hdd_get_pmkid_modes() - returns PMKID mode bits
 * @hdd_ctx: the pointer to hdd context
 * @pmkid_modes: struct to update with current PMKID modes
 *
 * Return: value of pmkid_modes
 */
void hdd_get_pmkid_modes(struct hdd_context *hdd_ctx,
			 struct pmkid_mode_bits *pmkid_modes)
{
	uint32_t cur_pmkid_modes;
	QDF_STATUS status;

	status = ucfg_mlme_get_pmkid_modes(hdd_ctx->psoc, &cur_pmkid_modes);
	if (status != QDF_STATUS_SUCCESS)
		hdd_err("get pmkid modes fail");

	pmkid_modes->fw_okc = (cur_pmkid_modes &
			       CFG_PMKID_MODES_OKC) ? 1 : 0;
	pmkid_modes->fw_pmksa_cache = (cur_pmkid_modes &
				       CFG_PMKID_MODES_PMKSA_CACHING) ? 1 : 0;
}

static void
hdd_populate_vdev_nss(struct wlan_mlme_nss_chains *user_cfg,
		      uint8_t tx_nss,
		      uint8_t rx_nss,
		      enum nss_chains_band_info  band)
{
	user_cfg->rx_nss[band] = rx_nss;
	user_cfg->tx_nss[band] = tx_nss;
}

static QDF_STATUS hdd_set_nss_params(struct wlan_hdd_link_info *link_info,
				     uint8_t tx_nss, uint8_t rx_nss)
{
	enum nss_chains_band_info band;
	struct wlan_mlme_nss_chains user_cfg;
	mac_handle_t mac_handle;
	struct hdd_adapter *adapter = link_info->adapter;
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	struct wlan_objmgr_vdev *vdev;

	qdf_mem_zero(&user_cfg, sizeof(user_cfg));

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_err("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_pdev(hdd_ctx->pdev,
						    link_info->vdev_id,
						    WLAN_HDD_ID_OBJ_MGR);
	if (!vdev) {
		hdd_err("vdev is NULL %d", link_info->vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	/* For STA tx/rx nss value is updated at the time of connection,
	 * for SAP case nss values will not get update, so can skip check
	 * for SAP/P2P_GO mode.
	 */
	if (adapter->device_mode != QDF_SAP_MODE &&
	    adapter->device_mode != QDF_P2P_GO_MODE &&
	    (tx_nss > wlan_vdev_mlme_get_nss(vdev) ||
	    rx_nss > wlan_vdev_mlme_get_nss(vdev))) {
		hdd_err("Given tx nss/rx nss is greater than intersected nss = %d",
			wlan_vdev_mlme_get_nss(vdev));
		wlan_objmgr_vdev_release_ref(vdev, WLAN_HDD_ID_OBJ_MGR);
		return QDF_STATUS_E_FAILURE;
	}
	wlan_objmgr_vdev_release_ref(vdev, WLAN_HDD_ID_OBJ_MGR);

	for (band = NSS_CHAINS_BAND_2GHZ; band < NSS_CHAINS_BAND_MAX; band++)
		hdd_populate_vdev_nss(&user_cfg, tx_nss, rx_nss, band);
	if (QDF_IS_STATUS_ERROR(
		sme_nss_chains_update(mac_handle, &user_cfg,
				      link_info->vdev_id)))
		return QDF_STATUS_E_FAILURE;

	return QDF_STATUS_SUCCESS;
}

static void hdd_update_nss_in_vdev(struct wlan_hdd_link_info *link_info,
				   mac_handle_t mac_handle, uint8_t tx_nss,
				   uint8_t rx_nss)
{
	uint8_t band, max_supp_nss = MAX_VDEV_NSS;
	struct wlan_objmgr_vdev *vdev;
	struct hdd_adapter *adapter = link_info->adapter;

	for (band = NSS_CHAINS_BAND_2GHZ; band < NSS_CHAINS_BAND_MAX;
	     band++) {
		/* This API will change the global ini in mlme cfg */
		sme_update_nss_in_mlme_cfg(mac_handle, rx_nss, tx_nss,
					   adapter->device_mode, band);
		/*
		 * This API will change the vdev nss params in mac
		 * context
		 */
		sme_update_vdev_type_nss(mac_handle, max_supp_nss, band);
	}
	/*
	 * This API will change the ini and dynamic nss params in
	 * mlme vdev priv obj.
	 */
	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return;

	hdd_store_nss_chains_cfg_in_vdev(adapter->hdd_ctx, vdev);
	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
}

static void hdd_set_sap_nss_params(struct wlan_hdd_link_info *link_info,
				   mac_handle_t mac_handle,
				   uint8_t tx_nss, uint8_t rx_nss)
{
	hdd_update_nss_in_vdev(link_info, mac_handle, tx_nss, rx_nss);
	hdd_restart_sap(link_info);
}

/**
 * hdd_get_sap_rx_nss() - get the sap rx nss
 * @link_info: Pointer to link_info
 * @rx_nss: pointer to rx_nss
 *
 * get the sap tx nss
 *
 * Return: None
 */
static QDF_STATUS
hdd_get_sap_rx_nss(struct wlan_hdd_link_info *link_info, uint8_t *rx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	struct wlan_mlme_nss_chains *dynamic_cfg;
	enum band_info operating_band;
	mac_handle_t mac_handle;
	uint8_t vdev_nss;

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_debug("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	operating_band = hdd_get_sap_operating_band_by_link_info(link_info);
	if (operating_band == BAND_UNKNOWN)
		return QDF_STATUS_E_INVAL;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	sme_get_sap_vdev_type_nss(mac_handle, &vdev_nss, operating_band);
	if (hdd_ctx->dynamic_nss_chains_support) {
		dynamic_cfg = mlme_get_dynamic_vdev_config(vdev);
		if (!dynamic_cfg) {
			hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
			hdd_debug("nss chain dynamic config NULL");
			return QDF_STATUS_E_INVAL;
		}
		switch (operating_band) {
		case BAND_2G:
			*rx_nss = dynamic_cfg->rx_nss[NSS_CHAINS_BAND_2GHZ];
			break;
		case BAND_5G:
			*rx_nss = dynamic_cfg->rx_nss[NSS_CHAINS_BAND_5GHZ];
			break;
		default:
			hdd_debug("Band %d Not 2G or 5G", operating_band);
			break;
		}
	} else {
		*rx_nss = vdev_nss;
	}

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
	return  QDF_STATUS_SUCCESS;
}

/**
 * hdd_get_sap_tx_nss() - get the sap tx nss
 * @link_info: Pointer of link_info
 * @tx_nss: pointer to tx_nss
 *
 * get the sap tx nss
 *
 * Return: None
 */
static QDF_STATUS
hdd_get_sap_tx_nss(struct wlan_hdd_link_info *link_info, uint8_t *tx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	struct wlan_mlme_nss_chains *dynamic_cfg;
	enum band_info operating_band;
	mac_handle_t mac_handle;
	uint8_t vdev_nss;

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_debug("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	operating_band = hdd_get_sap_operating_band_by_link_info(link_info);
	if (operating_band == BAND_UNKNOWN)
		return QDF_STATUS_E_INVAL;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	sme_get_sap_vdev_type_nss(mac_handle, &vdev_nss, operating_band);
	if (hdd_ctx->dynamic_nss_chains_support) {
		dynamic_cfg = mlme_get_dynamic_vdev_config(vdev);
		if (!dynamic_cfg) {
			hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
			hdd_debug("nss chain dynamic config NULL");
			return QDF_STATUS_E_INVAL;
		}
		switch (operating_band) {
		case BAND_2G:
			*tx_nss = dynamic_cfg->tx_nss[NSS_CHAINS_BAND_2GHZ];
			break;
		case BAND_5G:
			*tx_nss = dynamic_cfg->tx_nss[NSS_CHAINS_BAND_5GHZ];
			break;
		default:
			hdd_debug("Band %d Not 2G or 5G", operating_band);
			break;
		}
	} else {
		*tx_nss = vdev_nss;
	}

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
	return  QDF_STATUS_SUCCESS;
}

static bool
hdd_get_sap_restart_required_for_nss(struct wlan_hdd_link_info *link_info,
				     uint8_t tx_nss, uint8_t rx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	uint8_t rx_prev, tx_prev;
	bool restart_sap = 0;

	ucfg_mlme_get_restart_sap_on_dynamic_nss_chains_cfg(hdd_ctx->psoc,
							    &restart_sap);

	if (!restart_sap)
		return false;

	hdd_get_sap_rx_nss(link_info, &rx_prev);
	hdd_get_sap_tx_nss(link_info, &tx_prev);

	if (rx_prev != rx_nss && tx_prev != tx_nss)
		return true;
	return false;
}

QDF_STATUS hdd_update_nss(struct wlan_hdd_link_info *link_info,
			  uint8_t tx_nss, uint8_t rx_nss)
{
	struct hdd_adapter *adapter = link_info->adapter;
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	uint32_t rx_supp_data_rate, tx_supp_data_rate;
	bool status = true;
	QDF_STATUS qdf_status;
	qdf_size_t val_len;
	struct mlme_ht_capabilities_info ht_cap_info;
	uint8_t mcs_set[SIZE_OF_SUPPORTED_MCS_SET] = {0};
	uint8_t mcs_set_temp[SIZE_OF_SUPPORTED_MCS_SET];
	uint8_t enable2x2;
	mac_handle_t mac_handle;
	bool bval = 0, restart_sap = 0;

	if ((tx_nss == 2 || rx_nss == 2) && (hdd_ctx->num_rf_chains != 2)) {
		hdd_err("No support for 2 spatial streams");
		return QDF_STATUS_E_INVAL;
	}

	if (tx_nss > MAX_VDEV_NSS || rx_nss > MAX_VDEV_NSS) {
		hdd_debug("Cannot support tx_nss: %d rx_nss: %d", tx_nss,
			  rx_nss);
		return QDF_STATUS_E_INVAL;
	}

	qdf_status = ucfg_mlme_get_vht_enable2x2(hdd_ctx->psoc, &bval);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("unable to get vht_enable2x2");
		return QDF_STATUS_E_FAILURE;
	}

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_err("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	/*
	 * If FW is supporting the dynamic nss update, this command is meant to
	 * be per vdev, so update only the ini params of that particular vdev
	 * and not the global param enable2x2
	 */
	if (hdd_ctx->dynamic_nss_chains_support) {
		restart_sap =
		hdd_get_sap_restart_required_for_nss(link_info, tx_nss, rx_nss);

		if ((adapter->device_mode == QDF_SAP_MODE ||
		     adapter->device_mode == QDF_P2P_GO_MODE) && restart_sap) {
			if ((tx_nss == 2 && rx_nss == 2) ||
			    (tx_nss == 1 && rx_nss == 1)) {
				hdd_set_sap_nss_params(link_info, mac_handle,
						       tx_nss, rx_nss);
				return QDF_STATUS_SUCCESS;
			}
			hdd_err("tx_nss %d rx_nss %d not supported ",
				tx_nss, rx_nss);
			return QDF_STATUS_E_FAILURE;
		}

		if (hdd_is_vdev_in_conn_state(link_info))
			return hdd_set_nss_params(link_info, tx_nss, rx_nss);

		if (tx_nss != rx_nss) {
			hdd_err("TX NSS = %d, RX NSS  = %d value mismatch, doesn't support asymmetric config in disconnected state",
				tx_nss, rx_nss);
			return QDF_STATUS_E_FAILURE;
		}
		hdd_debug("Vdev %d in disconnect state, changing ini nss params",
			  link_info->vdev_id);
		if (!bval) {
			hdd_err("Nss in 1x1, no change required, 2x2 mode disabled");
			return QDF_STATUS_SUCCESS;
		}

		hdd_update_nss_in_vdev(link_info, mac_handle, tx_nss, rx_nss);
		sme_set_nss_capability(mac_handle, link_info->vdev_id,
				       rx_nss, adapter->device_mode);

		return QDF_STATUS_SUCCESS;
	}

	/*
	 * The code below is executed only when fw doesn't support dynamic
	 * update of nss and chains per vdev feature, for the upcoming
	 * connection
	 */
	enable2x2 = (rx_nss == 2) ? 1 : 0;

	if (bval == enable2x2) {
		hdd_debug("NSS same as requested");
		return QDF_STATUS_SUCCESS;
	}

	if (sme_is_any_session_in_connected_state(mac_handle)) {
		hdd_err("Connected sessions present, Do not change NSS");
		return QDF_STATUS_E_INVAL;
	}

	qdf_status = ucfg_mlme_set_vht_enable2x2(hdd_ctx->psoc, enable2x2);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set vht_enable2x2");
		return QDF_STATUS_E_FAILURE;
	}

	if (tx_nss == 1 && rx_nss == 2) {
		/* 1x2 */
		rx_supp_data_rate = VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_2_2;
		tx_supp_data_rate = VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
	} else if (enable2x2) {
		/* 2x2 */
		rx_supp_data_rate = VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_2_2;
		tx_supp_data_rate = VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_2_2;
	} else {
		/* 1x1 */
		rx_supp_data_rate = VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
		tx_supp_data_rate = VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
	}

	/* Update Rx Highest Long GI data Rate */
	qdf_status =
		ucfg_mlme_cfg_set_vht_rx_supp_data_rate(hdd_ctx->psoc,
							rx_supp_data_rate);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set rx_supp_data_rate");
		status = false;
	}
	/* Update Tx Highest Long GI data Rate */
	qdf_status =
		ucfg_mlme_cfg_set_vht_tx_supp_data_rate(hdd_ctx->psoc,
							tx_supp_data_rate);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set tx_supp_data_rate");
		status = false;
	}

	qdf_status = ucfg_mlme_get_ht_cap_info(hdd_ctx->psoc, &ht_cap_info);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to get HT Cap info");
		goto skip_ht_cap_update;
	}

	if (!(hdd_ctx->ht_tx_stbc_supported && enable2x2)) {
		ht_cap_info.tx_stbc = 0;
	} else {
		qdf_status =
			ucfg_mlme_cfg_get_vht_tx_stbc(hdd_ctx->psoc, &bval);
		if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
			hdd_err("Failed to get vht_tx_stbc");
			ht_cap_info.tx_stbc = bval;
		}
	}

	qdf_status = ucfg_mlme_set_ht_cap_info(hdd_ctx->psoc, ht_cap_info);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Could not set the HT_CAP_INFO");
	}
skip_ht_cap_update:
	qdf_status = ucfg_mlme_update_nss_vht_cap(hdd_ctx->psoc);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set update_nss_vht_cap");
		status = false;
	}

#define WLAN_HDD_RX_MCS_ALL_NSTREAM_RATES 0xff
	val_len = SIZE_OF_SUPPORTED_MCS_SET;
	qdf_status = ucfg_mlme_get_supported_mcs_set(hdd_ctx->psoc,
						     mcs_set_temp,
						     &val_len);
	if (QDF_IS_STATUS_SUCCESS(qdf_status)) {
		mcs_set[0] = mcs_set_temp[0];
		if (enable2x2)
			for (val_len = 0; val_len < rx_nss; val_len++)
				mcs_set[val_len] =
				WLAN_HDD_RX_MCS_ALL_NSTREAM_RATES;
		if (ucfg_mlme_set_supported_mcs_set(
			hdd_ctx->psoc, mcs_set,
			(qdf_size_t)SIZE_OF_SUPPORTED_MCS_SET) ==
			QDF_STATUS_E_FAILURE) {
			status = false;
			hdd_err("Could not pass on MCS SET to CFG");
		}
	} else {
		status = false;
		hdd_err("Could not get MCS SET from CFG");
	}
	sme_set_nss_capability(mac_handle, link_info->vdev_id,
			       rx_nss, adapter->device_mode);
#undef WLAN_HDD_RX_MCS_ALL_NSTREAM_RATES

	if (QDF_STATUS_SUCCESS != sme_update_nss(mac_handle, rx_nss))
		status = false;

	hdd_set_policy_mgr_user_cfg(hdd_ctx);

	return (status == false) ? QDF_STATUS_E_FAILURE : QDF_STATUS_SUCCESS;
}

QDF_STATUS hdd_get_nss(struct hdd_adapter *adapter, uint8_t *nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	bool bval;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	/*
	 * If FW is supporting the dynamic nss update, this command is meant to
	 * be per vdev, so get nss in the ini params of that particular vdev
	 * otherwise get it from the global param enable2x2
	 */
	if (hdd_ctx->dynamic_nss_chains_support) {
		uint8_t nss_2g, nss_5g;

		sme_get_vdev_type_nss(adapter->device_mode, &nss_2g, &nss_5g);
		/* Different settings in 2G and 5G is not supported */
		*nss = nss_2g;
	} else {
		status = ucfg_mlme_get_vht_enable2x2(hdd_ctx->psoc, &bval);
		if (!QDF_IS_STATUS_SUCCESS(status)) {
			hdd_err("unable to get vht_enable2x2");
			return status;
		}

		*nss = (bval) ? 2 : 1;
		if (!policy_mgr_is_hw_dbs_2x2_capable(hdd_ctx->psoc) &&
		    policy_mgr_is_current_hwmode_dbs(hdd_ctx->psoc))
			*nss = *nss - 1;
	}

	return status;
}

/**
 * hdd_get_sap_num_tx_chains() - get the sap num tx chains
 * @link_info: Pointer of link_info
 * @tx_chains: pointer to tx_chains
 *
 * get the sap num tx chains
 *
 * Return: None
 */
static QDF_STATUS
hdd_get_sap_num_tx_chains(struct wlan_hdd_link_info *link_info,
			  uint8_t *tx_chains)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	struct wlan_mlme_nss_chains *dynamic_cfg;
	enum band_info operating_band;
	mac_handle_t mac_handle;

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_debug("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	operating_band = hdd_get_sap_operating_band_by_link_info(link_info);
	if (operating_band == BAND_UNKNOWN)
		return QDF_STATUS_E_INVAL;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	if (hdd_ctx->dynamic_nss_chains_support) {
		dynamic_cfg = mlme_get_dynamic_vdev_config(vdev);
		if (!dynamic_cfg) {
			hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
			hdd_debug("nss chain dynamic config NULL");
			return QDF_STATUS_E_INVAL;
		}
		switch (operating_band) {
		case BAND_2G:
			*tx_chains =
			dynamic_cfg->num_tx_chains[NSS_CHAINS_BAND_2GHZ];
			break;
		case BAND_5G:
			*tx_chains =
			dynamic_cfg->num_tx_chains[NSS_CHAINS_BAND_5GHZ];
			break;
		default:
			hdd_debug("Band %d Not 2G or 5G", operating_band);
			break;
		}
	}

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
	return  QDF_STATUS_SUCCESS;
}

/**
 * hdd_get_sta_num_tx_chains() - get the sta num tx chains
 * @link_info: Pointer of link_info
 * @tx_chains: pointer to tx_chains
 *
 * get the STA num tx chains
 *
 * Return: None
 */
static QDF_STATUS
hdd_get_sta_num_tx_chains(struct wlan_hdd_link_info *link_info,
			  uint8_t *tx_chains)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	status = ucfg_mlme_get_sta_num_tx_chains(hdd_ctx->psoc, vdev,
						 tx_chains);
	if (QDF_IS_STATUS_ERROR(status))
		hdd_err("Failed to get sta_tx_nss");

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);

	return status;
}

/**
 * hdd_get_sta_tx_nss() - get the sta tx nss
 * @link_info: Pointer of link_info
 * @tx_nss: pointer to tx_nss
 *
 * get the STA tx nss
 *
 * Return: None
 */
static QDF_STATUS
hdd_get_sta_tx_nss(struct wlan_hdd_link_info *link_info, uint8_t *tx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	status = ucfg_mlme_get_sta_tx_nss(hdd_ctx->psoc, vdev, tx_nss);
	if (QDF_IS_STATUS_ERROR(status))
		hdd_err("Failed to get sta_tx_nss");

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);

	return status;
}

QDF_STATUS hdd_get_num_tx_chains(struct wlan_hdd_link_info *link_info,
				 uint8_t *tx_chains)
{
	struct hdd_adapter *adapter = link_info->adapter;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (adapter->device_mode == QDF_SAP_MODE ||
	    adapter->device_mode == QDF_P2P_GO_MODE)
		status = hdd_get_sap_num_tx_chains(link_info, tx_chains);
	else
		status = hdd_get_sta_num_tx_chains(link_info, tx_chains);

	return status;
}

QDF_STATUS hdd_get_tx_nss(struct wlan_hdd_link_info *link_info, uint8_t *tx_nss)
{
	struct hdd_adapter *adapter = link_info->adapter;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (adapter->device_mode == QDF_SAP_MODE ||
	    adapter->device_mode == QDF_P2P_GO_MODE)
		status = hdd_get_sap_tx_nss(link_info, tx_nss);
	else
		status = hdd_get_sta_tx_nss(link_info, tx_nss);

	return status;
}

/**
 * hdd_get_sap_num_rx_chains() - get the sap num rx chains
 * @link_info: Pointer to link_info
 * @rx_chains: pointer to rx_chains
 *
 * get the sap num rx chains
 *
 * Return: None
 */
static QDF_STATUS
hdd_get_sap_num_rx_chains(struct wlan_hdd_link_info *link_info,
			  uint8_t *rx_chains)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	struct wlan_mlme_nss_chains *dynamic_cfg;
	enum band_info operating_band;
	mac_handle_t mac_handle;

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_debug("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	operating_band = hdd_get_sap_operating_band_by_link_info(link_info);
	if (operating_band == BAND_UNKNOWN)
		return QDF_STATUS_E_INVAL;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	if (hdd_ctx->dynamic_nss_chains_support) {
		dynamic_cfg = mlme_get_dynamic_vdev_config(vdev);
		if (!dynamic_cfg) {
			hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
			hdd_debug("nss chain dynamic config NULL");
			return QDF_STATUS_E_INVAL;
		}
		switch (operating_band) {
		case BAND_2G:
			*rx_chains =
			dynamic_cfg->num_rx_chains[NSS_CHAINS_BAND_2GHZ];
			break;
		case BAND_5G:
			*rx_chains =
			dynamic_cfg->num_rx_chains[NSS_CHAINS_BAND_5GHZ];
			break;
		default:
			hdd_debug("Band %d Not 2G or 5G", operating_band);
			break;
		}
	}

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
	return  QDF_STATUS_SUCCESS;
}

/**
 * hdd_get_sta_num_rx_chains() - get the sta num rx chains
 * @link_info: Pointer to link_info in adapter
 * @rx_chains: pointer to rx_chains
 *
 * get the STA num rx chains
 *
 * Return: QDF_STATUS_SUCCESS if the RX NSS is returned, otherwise a suitable
 *         QDF_STATUS_E_* error code
 */
static QDF_STATUS
hdd_get_sta_num_rx_chains(struct wlan_hdd_link_info *link_info,
			  uint8_t *rx_chains)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	status = ucfg_mlme_get_sta_num_rx_chains(hdd_ctx->psoc, vdev,
						 rx_chains);
	if (QDF_IS_STATUS_ERROR(status))
		hdd_err("Failed to get sta_rx_nss");

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);

	return status;
}

/**
 * hdd_get_sta_rx_nss() - get the sta rx nss
 * @link_info: Pointer to link_info in adapter
 * @rx_nss: pointer to rx_nss
 *
 * get the STA rx nss
 *
 * Return: QDF_STATUS_SUCCESS if the RX NSS is returned, otherwise a suitable
 *         QDF_STATUS_E_* error code
 */
static QDF_STATUS
hdd_get_sta_rx_nss(struct wlan_hdd_link_info *link_info, uint8_t *rx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	status = wlan_mlme_get_sta_rx_nss(hdd_ctx->psoc, vdev, rx_nss);
	if (QDF_IS_STATUS_ERROR(status))
		hdd_err("Failed to get sta_rx_nss");

	hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);

	return status;
}

QDF_STATUS hdd_get_num_rx_chains(struct wlan_hdd_link_info *link_info,
				 uint8_t *rx_chains)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct hdd_adapter *adapter = link_info->adapter;

	if (adapter->device_mode == QDF_SAP_MODE ||
	    adapter->device_mode == QDF_P2P_GO_MODE)
		status = hdd_get_sap_num_rx_chains(link_info, rx_chains);
	else
		status = hdd_get_sta_num_rx_chains(link_info, rx_chains);

	return status;
}

QDF_STATUS hdd_get_rx_nss(struct wlan_hdd_link_info *link_info, uint8_t *rx_nss)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct hdd_adapter *adapter = link_info->adapter;

	if (adapter->device_mode == QDF_SAP_MODE ||
	    adapter->device_mode == QDF_P2P_GO_MODE)
		status = hdd_get_sap_rx_nss(link_info, rx_nss);
	else
		status = hdd_get_sta_rx_nss(link_info, rx_nss);

	return status;
}

int hdd_phymode_to_vendor_mode(eCsrPhyMode csr_phy_mode,
			       enum qca_wlan_vendor_phy_mode *vendor_phy_mode)
{
	switch (csr_phy_mode) {
	case eCSR_DOT11_MODE_AUTO:
	case eCSR_DOT11_MODE_11be:
	case eCSR_DOT11_MODE_11be_ONLY:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_AUTO;
		break;
	case eCSR_DOT11_MODE_11a:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_11A;
		break;
	case eCSR_DOT11_MODE_11b:
	case eCSR_DOT11_MODE_11b_ONLY:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_11B;
		break;
	case eCSR_DOT11_MODE_11g:
	case eCSR_DOT11_MODE_11g_ONLY:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_11G;
		break;
	case eCSR_DOT11_MODE_11n:
	case eCSR_DOT11_MODE_11n_ONLY:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_11AGN;
		break;
	case eCSR_DOT11_MODE_11ac:
	case eCSR_DOT11_MODE_11ac_ONLY:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT160;
		break;
	case eCSR_DOT11_MODE_11ax:
	case eCSR_DOT11_MODE_11ax_ONLY:
		*vendor_phy_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE160;
		break;
	case eCSR_DOT11_MODE_abg:
	default:
		hdd_err("Not supported mode %d", csr_phy_mode);
		return -EINVAL;
	}

	return 0;
}

int hdd_vendor_mode_to_phymode(enum qca_wlan_vendor_phy_mode vendor_phy_mode,
			       eCsrPhyMode *csr_phy_mode)
{
	switch (vendor_phy_mode) {
	case QCA_WLAN_VENDOR_PHY_MODE_AUTO:
	case QCA_WLAN_VENDOR_PHY_MODE_2G_AUTO:
	case QCA_WLAN_VENDOR_PHY_MODE_5G_AUTO:
		*csr_phy_mode = eCSR_DOT11_MODE_AUTO;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11A:
		*csr_phy_mode = eCSR_DOT11_MODE_11a;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11B:
		*csr_phy_mode = eCSR_DOT11_MODE_11b;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11G:
		*csr_phy_mode = eCSR_DOT11_MODE_11g;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AGN:
		*csr_phy_mode = eCSR_DOT11_MODE_11n;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80P80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT160:
		*csr_phy_mode = eCSR_DOT11_MODE_11ac;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE20:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80P80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE160:
		*csr_phy_mode = eCSR_DOT11_MODE_11ax;
		break;
	default:
		hdd_err("Not supported mode %d", vendor_phy_mode);
		return -EINVAL;
	}

	return 0;
}

int hdd_vendor_mode_to_band(enum qca_wlan_vendor_phy_mode vendor_phy_mode,
			    uint8_t *supported_band, bool is_6ghz_supported)
{
	switch (vendor_phy_mode) {
	case QCA_WLAN_VENDOR_PHY_MODE_AUTO:
		if (is_6ghz_supported)
			*supported_band = REG_BAND_MASK_ALL;
		else
			*supported_band =
				BIT(REG_BAND_2G) | BIT(REG_BAND_5G);
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80P80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT160:
		*supported_band = BIT(REG_BAND_2G) | BIT(REG_BAND_5G);
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE20:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80P80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE160:
	case QCA_WLAN_VENDOR_PHY_MODE_11AGN:
		if (is_6ghz_supported)
			*supported_band = REG_BAND_MASK_ALL;
		else
			*supported_band = BIT(REG_BAND_2G) | BIT(REG_BAND_5G);
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11A:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_5G_AUTO:
		*supported_band = BIT(REG_BAND_5G);
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11B:
	case QCA_WLAN_VENDOR_PHY_MODE_11G:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_2G_AUTO:
		*supported_band = BIT(REG_BAND_2G);
		break;
	default:
		hdd_err("Not supported mode %d", vendor_phy_mode);
		return -EINVAL;
	}

	return 0;
}

int
hdd_vendor_mode_to_bonding_mode(enum qca_wlan_vendor_phy_mode vendor_phy_mode,
				uint32_t *bonding_mode)
{
	switch (vendor_phy_mode) {
	case QCA_WLAN_VENDOR_PHY_MODE_AUTO:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80P80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT160:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40PLUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40MINUS:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80P80:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE160:
	case QCA_WLAN_VENDOR_PHY_MODE_2G_AUTO:
	case QCA_WLAN_VENDOR_PHY_MODE_5G_AUTO:
	case QCA_WLAN_VENDOR_PHY_MODE_11AGN:
		*bonding_mode = WNI_CFG_CHANNEL_BONDING_MODE_ENABLE;
		break;
	case QCA_WLAN_VENDOR_PHY_MODE_11A:
	case QCA_WLAN_VENDOR_PHY_MODE_11B:
	case QCA_WLAN_VENDOR_PHY_MODE_11G:
	case QCA_WLAN_VENDOR_PHY_MODE_11NA_HT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11NG_HT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT20:
	case QCA_WLAN_VENDOR_PHY_MODE_11AX_HE20:
		*bonding_mode = WNI_CFG_CHANNEL_BONDING_MODE_DISABLE;
		break;
	default:
		hdd_err("Not supported mode %d", vendor_phy_mode);
		return -EINVAL;
	}

	return 0;
}

int hdd_phymode_to_dot11_mode(eCsrPhyMode phymode,
			      enum hdd_dot11_mode *dot11_mode)
{
	switch (phymode) {
	case eCSR_DOT11_MODE_AUTO:
	case eCSR_DOT11_MODE_11be:
		*dot11_mode = eHDD_DOT11_MODE_AUTO;
		break;
	case eCSR_DOT11_MODE_11a:
		*dot11_mode = eHDD_DOT11_MODE_11a;
		break;
	case eCSR_DOT11_MODE_11b:
		*dot11_mode = eHDD_DOT11_MODE_11b;
		break;
	case eCSR_DOT11_MODE_11g:
		*dot11_mode = eHDD_DOT11_MODE_11g;
		break;
	case eCSR_DOT11_MODE_11n:
		*dot11_mode = eHDD_DOT11_MODE_11n;
		break;
	case eCSR_DOT11_MODE_11ac:
		*dot11_mode = eHDD_DOT11_MODE_11ac;
		break;
	case eCSR_DOT11_MODE_11ax:
		*dot11_mode = eHDD_DOT11_MODE_11ax;
		break;
	default:
		hdd_err("Not supported mode %d", phymode);
		return -EINVAL;
	}

	return 0;
}

#ifdef QCA_HT_2040_COEX
static QDF_STATUS
hdd_set_ht2040_mode(struct hdd_adapter *adapter,
		    struct csr_config_params *csr_config,
		    uint32_t bonding_mode)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (csr_config->phyMode == eCSR_DOT11_MODE_11n) {
		if (bonding_mode == WNI_CFG_CHANNEL_BONDING_MODE_ENABLE)
			csr_config->obssEnabled = true;
		else
			csr_config->obssEnabled = false;
		status = sme_set_ht2040_mode(hdd_ctx->mac_handle,
					     adapter->deflink->vdev_id,
					     eHT_CHAN_HT20,
					     csr_config->obssEnabled);
	}

	return status;
}
#else
static QDF_STATUS
hdd_set_ht2040_mode(struct hdd_adapter *adapter,
		    struct csr_config_params *csr_config,
		    uint32_t bonding_mode)
{
	return QDF_STATUS_SUCCESS;
}
#endif

int hdd_update_phymode(struct hdd_adapter *adapter, eCsrPhyMode phymode,
		       uint8_t supported_band, uint32_t bonding_mode)
{
	struct net_device *net = adapter->dev;
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	struct sme_config_params *sme_config = NULL;
	struct csr_config_params *csr_config;
	eCsrPhyMode old_phymode;
	enum hdd_dot11_mode hdd_dot11mode;
	int ret = 0;
	QDF_STATUS status;

	ret = wlan_hdd_validate_context(hdd_ctx);
	if (ret < 0)
		return ret;

	ret = hdd_phymode_to_dot11_mode(phymode, &hdd_dot11mode);
	if (ret < 0)
		return ret;

	hdd_debug("phymode=%d bonding_mode=%d supported_band=%d",
		  phymode, bonding_mode, supported_band);

	old_phymode = sme_get_phy_mode(hdd_ctx->mac_handle);

	sme_set_phy_mode(hdd_ctx->mac_handle, phymode);

	if (hdd_reg_set_band(net, supported_band)) {
		sme_set_phy_mode(hdd_ctx->mac_handle, old_phymode);
		return -EIO;
	}

	sme_config = qdf_mem_malloc(sizeof(*sme_config));
	if (!sme_config)
		return -ENOMEM;

	sme_get_config_param(hdd_ctx->mac_handle, sme_config);
	csr_config = &sme_config->csr_config;
	csr_config->phyMode = phymode;

	status = hdd_set_ht2040_mode(adapter, csr_config, bonding_mode);
	if (QDF_IS_STATUS_ERROR(status)) {
		hdd_err("Failed to set ht2040 mode");
		ret = -EIO;
		goto free;
	}

	status = ucfg_mlme_set_band_capability(hdd_ctx->psoc, supported_band);
	if (QDF_IS_STATUS_ERROR(status)) {
		hdd_err("failed to set MLME band capability");
		ret = -EIO;
		goto free;
	}

	if (supported_band == BIT(REG_BAND_2G)) {
		status = ucfg_mlme_set_11h_enabled(hdd_ctx->psoc, 0);
		if (!QDF_IS_STATUS_SUCCESS(status)) {
			hdd_err("Failed to set 11h_enable flag");
			ret = -EIO;
			goto free;
		}
	}
	if (supported_band & BIT(REG_BAND_2G))
		csr_config->channelBondingMode24GHz = bonding_mode;

	if (supported_band & BIT(REG_BAND_5G))
		csr_config->channelBondingMode5GHz = bonding_mode;

	sme_update_config(hdd_ctx->mac_handle, sme_config);

	hdd_ctx->config->dot11Mode = hdd_dot11mode;
	ucfg_mlme_set_channel_bonding_24ghz(hdd_ctx->psoc,
					   csr_config->channelBondingMode24GHz);
	ucfg_mlme_set_channel_bonding_5ghz(hdd_ctx->psoc,
					   csr_config->channelBondingMode5GHz);
	if (hdd_update_config_cfg(hdd_ctx) == false) {
		hdd_err("could not update config_dat");
		ret = -EIO;
		goto free;
	}

	if (supported_band & BIT(REG_BAND_5G)) {
		struct ieee80211_supported_band *ieee_band;
		uint32_t channel_bonding_mode;

		ucfg_mlme_get_channel_bonding_5ghz(hdd_ctx->psoc,
						   &channel_bonding_mode);
		ieee_band = hdd_ctx->wiphy->bands[HDD_NL80211_BAND_5GHZ];
		if (channel_bonding_mode)
			ieee_band->ht_cap.cap |=
					IEEE80211_HT_CAP_SUP_WIDTH_20_40;
		else
			ieee_band->ht_cap.cap &=
					~IEEE80211_HT_CAP_SUP_WIDTH_20_40;
	}

free:
	if (sme_config)
		qdf_mem_free(sme_config);
	return ret;
}

int hdd_get_ldpc(struct hdd_adapter *adapter, int *value)
{
	mac_handle_t mac_handle = adapter->hdd_ctx->mac_handle;
	int ret;

	hdd_enter();
	ret = sme_get_ht_config(mac_handle, adapter->deflink->vdev_id,
				WNI_CFG_HT_CAP_INFO_ADVANCE_CODING);
	if (ret < 0) {
		hdd_err("Failed to get LDPC value");
	} else {
		*value = ret;
		ret = 0;
	}
	return ret;
}

int hdd_set_ldpc(struct wlan_hdd_link_info *link_info, int value)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	mac_handle_t mac_handle = hdd_ctx->mac_handle;
	int ret;
	QDF_STATUS status;
	struct mlme_ht_capabilities_info ht_cap_info;

	hdd_debug("%d", value);

	if (!mac_handle) {
		hdd_err("NULL Mac handle");
		return -EINVAL;
	}

	status = ucfg_mlme_get_ht_cap_info(hdd_ctx->psoc, &ht_cap_info);
	if (QDF_IS_STATUS_ERROR(status)) {
		hdd_err("Failed to get HT capability info");
		return -EIO;
	}

	ht_cap_info.adv_coding_cap = value;
	status = ucfg_mlme_set_ht_cap_info(hdd_ctx->psoc, ht_cap_info);
	if (QDF_IS_STATUS_ERROR(status)) {
		hdd_err("Failed to set HT capability info");
		return -EIO;
	}
	status = ucfg_mlme_cfg_set_vht_ldpc_coding_cap(hdd_ctx->psoc, value);
	if (QDF_IS_STATUS_ERROR(status)) {
		hdd_err("Failed to set VHT LDPC capability info");
		return -EIO;
	}
	ret = sme_update_ht_config(mac_handle, link_info->vdev_id,
				   WNI_CFG_HT_CAP_INFO_ADVANCE_CODING, value);
	if (ret)
		hdd_err("Failed to set LDPC value");
	ret = sme_update_he_ldpc_supp(mac_handle,
				      link_info->vdev_id, value);
	if (ret)
		hdd_err("Failed to set HE LDPC value");
	ret = sme_set_auto_rate_ldpc(mac_handle, link_info->vdev_id,
				     (value ? 0 : 1));

	return ret;
}

int hdd_get_tx_stbc(struct hdd_adapter *adapter, int *value)
{
	mac_handle_t mac_handle = adapter->hdd_ctx->mac_handle;
	int ret;

	hdd_enter();
	ret = sme_get_ht_config(mac_handle, adapter->deflink->vdev_id,
				WNI_CFG_HT_CAP_INFO_TX_STBC);
	if (ret < 0) {
		hdd_err("Failed to get TX STBC value");
	} else {
		*value = ret;
		ret = 0;
	}

	return ret;
}

int hdd_set_tx_stbc(struct wlan_hdd_link_info *link_info, int value)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	mac_handle_t mac_handle = hdd_ctx->mac_handle;
	int ret;
	QDF_STATUS status;
	struct mlme_ht_capabilities_info ht_cap_info;

	hdd_debug("%d", value);

	if (!mac_handle) {
		hdd_err("NULL Mac handle");
		return -EINVAL;
	}

	if (value) {
		/* make sure HT capabilities allow this */
		status = ucfg_mlme_get_ht_cap_info(hdd_ctx->psoc,
						   &ht_cap_info);
		if (QDF_IS_STATUS_ERROR(status)) {
			hdd_err("Failed to get HT capability info");
			return -EIO;
		}
		if (!ht_cap_info.tx_stbc) {
			hdd_err("TX STBC not supported");
			return -EINVAL;
		}
	}
	ret = sme_update_ht_config(mac_handle, link_info->vdev_id,
				   WNI_CFG_HT_CAP_INFO_TX_STBC,
				   value);
	if (ret)
		hdd_err("Failed to set TX STBC value");
	ret = sme_update_he_tx_stbc_cap(mac_handle,
					link_info->vdev_id, value);
	if (ret)
		hdd_err("Failed to set HE TX STBC value");

	return ret;
}

int hdd_get_rx_stbc(struct hdd_adapter *adapter, int *value)
{
	mac_handle_t mac_handle = adapter->hdd_ctx->mac_handle;
	int ret;

	hdd_enter();
	ret = sme_get_ht_config(mac_handle, adapter->deflink->vdev_id,
				WNI_CFG_HT_CAP_INFO_RX_STBC);
	if (ret < 0) {
		hdd_err("Failed to get RX STBC value");
	} else {
		*value = ret;
		ret = 0;
	}

	return ret;
}

int hdd_set_rx_stbc(struct wlan_hdd_link_info *link_info, int value)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	mac_handle_t mac_handle = hdd_ctx->mac_handle;
	int ret;
	QDF_STATUS status;
	struct mlme_ht_capabilities_info ht_cap_info;

	hdd_debug("%d", value);

	if (!mac_handle) {
		hdd_err("NULL Mac handle");
		return -EINVAL;
	}

	if (value) {
		/* make sure HT capabilities allow this */
		status = ucfg_mlme_get_ht_cap_info(hdd_ctx->psoc,
						   &ht_cap_info);
		if (QDF_IS_STATUS_ERROR(status)) {
			hdd_err("Failed to get HT capability info");
			return -EIO;
		}
		if (!ht_cap_info.rx_stbc) {
			hdd_warn("RX STBC not supported");
			return -EINVAL;
		}
	}
	ret = sme_update_ht_config(mac_handle, link_info->vdev_id,
				   WNI_CFG_HT_CAP_INFO_RX_STBC,
				   value);
	if (ret)
		hdd_err("Failed to set RX STBC value");

	ret = sme_update_he_rx_stbc_cap(mac_handle,
					link_info->vdev_id, value);
	if (ret)
		hdd_err("Failed to set HE RX STBC value");

	return ret;
}

/**
 * hdd_convert_chwidth_to_phy_chwidth() - convert channel width of type enum
 * eSirMacHTChannelWidth to enum phy_ch_width
 * @chwidth: channel width of type enum eSirMacHTChannelWidth
 *
 * Return: channel width of type enum phy_ch_width
 */
static enum phy_ch_width
hdd_convert_chwidth_to_phy_chwidth(enum eSirMacHTChannelWidth chwidth)
{
	enum phy_ch_width ch_width = CH_WIDTH_INVALID;

	switch (chwidth) {
	case eHT_CHANNEL_WIDTH_20MHZ:
		ch_width = CH_WIDTH_20MHZ;
		break;
	case eHT_CHANNEL_WIDTH_40MHZ:
		ch_width = CH_WIDTH_40MHZ;
		break;
	case eHT_CHANNEL_WIDTH_80MHZ:
		ch_width = CH_WIDTH_80MHZ;
		break;
	case eHT_CHANNEL_WIDTH_160MHZ:
		ch_width = CH_WIDTH_160MHZ;
		break;
	case eHT_CHANNEL_WIDTH_80P80MHZ:
		ch_width = CH_WIDTH_80P80MHZ;
		break;
	case eHT_CHANNEL_WIDTH_320MHZ:
		ch_width = CH_WIDTH_320MHZ;
		break;
	default:
		hdd_debug("Invalid channel width %d", chwidth);
		break;
	}

	return ch_width;
}

/**
 * hdd_update_bss_rate_flags() - update bss rate flag as per new channel width
 * @link_info: Link info in HDD adapter
 * @psoc: psoc common object
 * @cw: channel width for which bss rate flag being updated
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS
hdd_update_bss_rate_flags(struct wlan_hdd_link_info *link_info,
			  struct wlan_objmgr_psoc *psoc, enum phy_ch_width cw)
{
	struct hdd_station_ctx *hdd_sta_ctx;
	uint8_t eht_present, he_present, vht_present, ht_present;

	hdd_sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(link_info);

	eht_present = hdd_sta_ctx->conn_info.conn_flag.eht_present;
	he_present = hdd_sta_ctx->conn_info.conn_flag.he_present;
	vht_present = hdd_sta_ctx->conn_info.conn_flag.vht_present;
	ht_present = hdd_sta_ctx->conn_info.conn_flag.ht_present;

	return ucfg_mlme_update_bss_rate_flags(psoc, link_info->vdev_id,
					       cw, eht_present, he_present,
					       vht_present, ht_present);
}

/**
 * struct sme_config_msg_ctx - sme config update message ctx
 * @vdev: vdev object
 * @chwidth: channel width
 * @is_restore: restore default or not
 * @bonding_mode: bonding mode
 */
struct sme_config_msg_ctx {
	struct wlan_objmgr_vdev *vdev;
	enum eSirMacHTChannelWidth chwidth;
	bool is_restore;
	uint32_t bonding_mode;
};

/**
 * hdd_restore_sme_config_cb() - restore bonding mode sme config cb
 * @msg: msg data
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS hdd_restore_sme_config_cb(struct scheduler_msg *msg)
{
	struct hdd_context *hdd_ctx;
	mac_handle_t mac_handle;
	struct sme_config_params *sme_config = NULL;
	struct sme_config_msg_ctx *sme_config_msg_ctx = NULL;
	struct wlan_objmgr_vdev *vdev = NULL;
	uint8_t vdev_id;
	QDF_STATUS status = QDF_STATUS_E_FAILURE;

	if (!msg) {
		hdd_debug("msg is null");
		return QDF_STATUS_E_INVAL;
	}

	sme_config_msg_ctx = msg->bodyptr;
	if (!sme_config_msg_ctx) {
		hdd_debug("bodyptr is null");
		return QDF_STATUS_E_INVAL;
	}

	vdev = sme_config_msg_ctx->vdev;
	if (!vdev)
		goto end;

	hdd_ctx = cds_get_context(QDF_MODULE_ID_HDD);
	if (wlan_hdd_validate_context(hdd_ctx))
		goto end;

	mac_handle = cds_get_context(QDF_MODULE_ID_SME);
	if (!mac_handle)
		goto end;

	if (!hdd_ctx->psoc)
		goto end;

	sme_config = qdf_mem_malloc(sizeof(*sme_config));
	if (!sme_config)
		goto end;

	vdev_id = wlan_vdev_get_id(vdev);
	hdd_debug("vdev id %d is_restore %d bonding_mode %d chwdith %d",
		  vdev_id,
		  sme_config_msg_ctx->is_restore,
		  sme_config_msg_ctx->bonding_mode,
		  sme_config_msg_ctx->chwidth);
	sme_get_config_param(mac_handle, sme_config);
	if (sme_config_msg_ctx->is_restore) {
		sme_config->csr_config.channelBondingMode5GHz =
			cfg_get(hdd_ctx->psoc, CFG_CHANNEL_BONDING_MODE_5GHZ);
		sme_config->csr_config.channelBondingMode24GHz =
			cfg_get(hdd_ctx->psoc, CFG_CHANNEL_BONDING_MODE_24GHZ);
	} else {
		sme_config->csr_config.channelBondingMode5GHz =
					sme_config_msg_ctx->bonding_mode;
		sme_config->csr_config.channelBondingMode24GHz =
					sme_config_msg_ctx->bonding_mode;
	}
	sme_update_config(mac_handle, sme_config);
	sme_set_he_bw_cap(hdd_ctx->mac_handle, vdev_id,
			  sme_config_msg_ctx->chwidth);
	sme_set_eht_bw_cap(hdd_ctx->mac_handle, vdev_id,
			   sme_config_msg_ctx->chwidth);

	status = QDF_STATUS_SUCCESS;
end:
	qdf_mem_free(sme_config);
	if (vdev)
		hdd_objmgr_put_vdev_by_user(vdev, WLAN_HDD_ID_OBJ_MGR);
	qdf_mem_free(sme_config_msg_ctx);

	return status;
}

/**
 * hdd_restore_sme_config_flush_cb() - bonding mode sme config flush cb
 * @msg: msg data
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS hdd_restore_sme_config_flush_cb(struct scheduler_msg *msg)
{
	struct sme_config_msg_ctx *sme_config_msg_ctx;

	if (!msg) {
		hdd_debug("msg is null");
		return QDF_STATUS_E_INVAL;
	}

	sme_config_msg_ctx = msg->bodyptr;
	if (!sme_config_msg_ctx) {
		hdd_debug("bodyptr is null");
		return QDF_STATUS_E_INVAL;
	}

	if (sme_config_msg_ctx->vdev)
		hdd_objmgr_put_vdev_by_user(sme_config_msg_ctx->vdev,
					    WLAN_HDD_ID_OBJ_MGR);
	else
		hdd_debug("vdev is null");

	qdf_mem_free(sme_config_msg_ctx);

	return QDF_STATUS_SUCCESS;
}

/**
 * hdd_restore_sme_config() - restore bonding mode for sme config
 * @link_info: link info
 * @chwidth: channel width
 * @is_restore: msg data
 * @bonding_mode: bonding mode
 *
 * Return: void
 */
static void hdd_restore_sme_config(struct wlan_hdd_link_info *link_info,
				   enum eSirMacHTChannelWidth chwidth,
				   bool is_restore, uint32_t bonding_mode)
{
	struct scheduler_msg msg = {0};
	QDF_STATUS status;
	struct wlan_objmgr_vdev *vdev;
	struct sme_config_msg_ctx *sme_config_msg_ctx;

	sme_config_msg_ctx = qdf_mem_malloc(sizeof(*sme_config_msg_ctx));
	if (!sme_config_msg_ctx)
		return;

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_HDD_ID_OBJ_MGR);
	if (!vdev) {
		qdf_mem_free(sme_config_msg_ctx);
		hdd_debug("no vdev from link info");
		return;
	}

	sme_config_msg_ctx->vdev = vdev;
	sme_config_msg_ctx->chwidth = chwidth;
	sme_config_msg_ctx->is_restore = is_restore;
	sme_config_msg_ctx->bonding_mode = bonding_mode;
	msg.bodyptr = sme_config_msg_ctx;
	msg.callback = hdd_restore_sme_config_cb;
	msg.flush_callback = hdd_restore_sme_config_flush_cb;

	status = scheduler_post_message(QDF_MODULE_ID_HDD,
					QDF_MODULE_ID_OS_IF,
					QDF_MODULE_ID_OS_IF, &msg);
	if (QDF_IS_STATUS_ERROR(status)) {
		hdd_debug("status %d", status);
		hdd_objmgr_put_vdev_by_user(vdev, WLAN_HDD_ID_OBJ_MGR);
		qdf_mem_free(sme_config_msg_ctx);
	}
}

/**
 * wlan_update_mlo_link_chn_width() - API to update mlo link chn width
 * @adapter: the pointer to adapter
 * @ch_width: channel width to update
 * @link_id: mlo link id
 *
 * Get link id and channel bandwidth from user space and save in link_info.
 * When link switch happen and host driver connect done, if the link change
 * from standby to non-standby, ch_width will send to fw again.
 *
 * Return: QDF_STATUS
 */

#if defined(WLAN_FEATURE_11BE_MLO) && defined(WLAN_HDD_MULTI_VDEV_SINGLE_NDEV)
static struct wlan_hdd_link_info *
wlan_update_mlo_link_chn_width(struct hdd_adapter *adapter,
			       enum phy_ch_width ch_width,
			       uint8_t link_id)
{
	struct wlan_hdd_link_info *link_info;
	struct hdd_station_ctx *sta_ctx;

	link_info = hdd_get_link_info_by_ieee_link_id(adapter, link_id, false);
	if (!link_info)
		return NULL;

	sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(link_info);

	sta_ctx->user_cfg_chn_width = ch_width;
	hdd_debug("save ch_width:%u to link_id:%u vdev_id:%u",
		  ch_width, link_id, link_info->vdev_id);

	return link_info;
}
#else
static struct wlan_hdd_link_info *
wlan_update_mlo_link_chn_width(struct hdd_adapter *adapter,
			       enum phy_ch_width ch_width,
			       uint8_t link_id)
{
	return NULL;
}
#endif

int hdd_update_channel_width(struct wlan_hdd_link_info *link_info,
			     enum eSirMacHTChannelWidth chwidth,
			     uint32_t bonding_mode, uint8_t link_id,
			     bool is_restore)
{
	struct hdd_context *hdd_ctx;
	int ret;
	enum phy_ch_width ch_width;
	struct wlan_objmgr_vdev *link_vdev;
	struct wlan_objmgr_vdev *vdev;
	struct wlan_hdd_link_info *link_info_t;
	uint8_t link_vdev_id;
	enum QDF_OPMODE op_mode;
	QDF_STATUS status;
	uint8_t vdev_id = link_info->vdev_id;
	enum phy_ch_width new_ch_width;

	hdd_ctx = WLAN_HDD_GET_CTX(link_info->adapter);
	if (!hdd_ctx) {
		hdd_err("hdd_ctx failure");
		return -EINVAL;
	}

	op_mode = link_info->adapter->device_mode;
	if (op_mode != QDF_STA_MODE) {
		hdd_debug("vdev %d: op mode %d, CW update not supported",
			  vdev_id, op_mode);
		return -EINVAL;
	}

	vdev = hdd_objmgr_get_vdev_by_user(link_info, WLAN_OSIF_ID);
	if (!vdev) {
		hdd_err("vdev %d: vdev not found", vdev_id);
		return -EINVAL;
	}

	ch_width = hdd_convert_chwidth_to_phy_chwidth(chwidth);

	/**
	 * Link_id check is for disconnect restore process.
	 * Disconnect will not update channel bandwidth into cache struct.
	 */
	if (wlan_vdev_mlme_is_mlo_vdev(vdev) &&
	    link_id != WLAN_INVALID_LINK_ID) {
		link_info_t = wlan_update_mlo_link_chn_width(link_info->adapter,
							     ch_width, link_id);
		if (!link_info_t) {
			hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);
			return -EINVAL;
		}

		hdd_objmgr_put_vdev_by_user(vdev, WLAN_OSIF_ID);

		link_vdev = hdd_objmgr_get_vdev_by_user(link_info_t,
							WLAN_OSIF_ID);
		if (!link_vdev)
			return 0;

		link_vdev_id = link_info_t->vdev_id;
		status = wlan_mlme_get_bw_no_punct(hdd_ctx->psoc,
						   link_vdev,
						   wlan_vdev_mlme_get_des_chan(link_vdev),
						   &new_ch_width);
		if (QDF_IS_STATUS_SUCCESS(status) && ch_width > new_ch_width)
			ch_width = new_ch_width;
	} else {
		link_vdev = vdev;
		link_vdev_id = vdev_id;
		link_info_t = link_info;
	}

	if (ucfg_mlme_is_chwidth_with_notify_supported(hdd_ctx->psoc) &&
	    hdd_cm_is_vdev_connected(link_info_t)) {
		ch_width = hdd_convert_chwidth_to_phy_chwidth(chwidth);
		hdd_debug("vdev %d : process update ch width request to %d",
			  link_vdev_id, ch_width);
		status = ucfg_mlme_send_ch_width_update_with_notify(hdd_ctx->psoc,
								    link_vdev,
								    ch_width,
								    link_vdev_id);

		if (QDF_IS_STATUS_ERROR(status)) {
			hdd_objmgr_put_vdev_by_user(link_vdev, WLAN_OSIF_ID);
			return -EIO;
		}
		status = hdd_update_bss_rate_flags(link_info_t, hdd_ctx->psoc,
						   ch_width);
		if (QDF_IS_STATUS_ERROR(status)) {
			hdd_objmgr_put_vdev_by_user(link_vdev, WLAN_OSIF_ID);
			return -EIO;
		}

		hdd_objmgr_put_vdev_by_user(link_vdev, WLAN_OSIF_ID);
		return 0;
	}
	hdd_objmgr_put_vdev_by_user(link_vdev, WLAN_OSIF_ID);

	ret = wma_cli_set_command(link_vdev_id, wmi_vdev_param_chwidth,
				  chwidth, VDEV_CMD);
	if (ret)
		return ret;

	hdd_restore_sme_config(link_info_t, chwidth, is_restore, bonding_mode);

	return 0;
}
