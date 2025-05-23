/*
 * Copyright (c) 2013-2021 The Linux Foundation. All rights reserved.
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

#include "targaddrs.h"
#include "target_type.h"
#include "cepci.h"
#include "regtable.h"
#include "ar6320def.h"
#include "ar6320v2def.h"
#include "hif_main.h"
#include "adrastea_reg_def.h"
#include "wcn6450def.h"

#include "targetdef.h"
#include "hostdef.h"

void hif_target_register_tbl_attach(struct hif_softc *scn, u32 target_type)
{
	switch (target_type) {
	case TARGET_TYPE_AR6320:
		scn->targetdef = &ar6320_targetdef;
		scn->target_ce_def = &ar6320_ce_targetdef;
		break;
	case TARGET_TYPE_AR6320V2:
		scn->targetdef = &ar6320v2_targetdef;
		scn->target_ce_def = &ar6320v2_ce_targetdef;
		break;
	case TARGET_TYPE_ADRASTEA:
		scn->targetdef = &adrastea_targetdef;
		scn->target_ce_def = &adrastea_ce_targetdef;
		break;
	case TARGET_TYPE_QCN7605:
		scn->targetdef = &genoa_targetdef;
		scn->target_ce_def = &genoa_ce_targetdef;
		break;
#if defined(AR6002_HEADERS_DEF)
	case TARGET_TYPE_AR6002:
		scn->targetdef = AR6002_TARGETdef;
		break;
#endif
#if defined(AR6003_HEADERS_DEF)
	case TARGET_TYPE_AR6003:
		scn->targetdef = AR6003_TARGETdef;
		break;
#endif
#if defined(AR6004_HEADERS_DEF)
	case TARGET_TYPE_AR6004:
		scn->targetdef = AR6004_TARGETdef;
		break;
#endif
#if defined(AR9888_HEADERS_DEF)
	case TARGET_TYPE_AR9888:
		scn->targetdef = AR9888_TARGETdef;
		scn->target_ce_def = AR9888_CE_TARGETdef;
		break;
#endif
#if defined(AR9888V2_HEADERS_DEF)
	case TARGET_TYPE_AR9888V2:
		scn->targetdef = AR9888V2_TARGETdef;
		scn->target_ce_def = AR9888_CE_TARGETdef;
		break;
#endif
#if defined(AR900B_HEADERS_DEF)
	case TARGET_TYPE_AR900B:
		scn->targetdef = AR900B_TARGETdef;
		scn->target_ce_def = AR900B_CE_TARGETdef;
		break;
#endif
#if defined(QCA9984_HEADERS_DEF)
	case TARGET_TYPE_QCA9984:
		scn->targetdef = QCA9984_TARGETdef;
		scn->target_ce_def = QCA9984_CE_TARGETdef;
		break;
#endif
#if defined(QCA9888_HEADERS_DEF)
	case TARGET_TYPE_QCA9888:
		scn->targetdef = QCA9888_TARGETdef;
		scn->target_ce_def = QCA9888_CE_TARGETdef;
		break;
#endif
#if defined(QCA8074_HEADERS_DEF)
	case TARGET_TYPE_QCA8074:
		scn->targetdef = QCA8074_TARGETdef;
		scn->target_ce_def = QCA8074_CE_TARGETdef;
		break;
#endif

#if defined(QCA6290_HEADERS_DEF)
	case TARGET_TYPE_QCA6290:
		scn->targetdef = QCA6290_TARGETdef;
		scn->target_ce_def = QCA6290_CE_TARGETdef;
		break;
#endif
#if defined(QCA8074V2_HEADERS_DEF)
	case TARGET_TYPE_QCA8074V2:
		scn->targetdef = QCA8074V2_TARGETDEF;
		scn->target_ce_def = QCA8074V2_CE_TARGETDEF;
		break;
#endif
#if defined(QCA6018_HEADERS_DEF)
	case TARGET_TYPE_QCA6018:
		scn->targetdef = QCA6018_TARGETDEF;
		scn->target_ce_def = QCA6018_CE_TARGETDEF;
		break;
#endif
#if defined(QCA9574_HEADERS_DEF)
	case TARGET_TYPE_QCA9574:
		scn->targetdef = QCA9574_TARGETDEF;
		scn->target_ce_def = QCA9574_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCA9574");
		break;
#endif

#if defined(QCN9000_HEADERS_DEF)
	case TARGET_TYPE_QCN9000:
		scn->targetdef = QCN9000_TARGETDEF;
		scn->target_ce_def = QCN9000_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCN9000");
		break;
#endif

#if defined(QCN9224_HEADERS_DEF)
	case TARGET_TYPE_QCN9224:
		scn->targetdef = QCN9224_TARGETDEF;
		scn->target_ce_def = QCN9224_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCN9224");
		break;
#endif

#if defined(QCN6122_HEADERS_DEF)
	case TARGET_TYPE_QCN6122:
		scn->targetdef = QCN6122_TARGETDEF;
		scn->target_ce_def = QCN6122_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCN6122");
		break;
#endif

#if defined(QCN9160_HEADERS_DEF)
	case TARGET_TYPE_QCN9160:
		scn->targetdef = QCN9160_TARGETDEF;
		scn->target_ce_def = QCN9160_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCN9160");
		break;
#endif

#if defined(QCN6432_HEADERS_DEF)
	case TARGET_TYPE_QCN6432:
		scn->targetdef = QCN6432_TARGETDEF;
		scn->target_ce_def = QCN6432_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCN6432");
		break;
#endif

#if defined(QCA5424_HEADERS_DEF)
	case TARGET_TYPE_QCA5424:
		scn->targetdef = QCA5424_TARGETDEF;
		scn->target_ce_def = QCA5424_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCA5424");
		break;
#endif

#if defined(QCA5018_HEADERS_DEF)
	case TARGET_TYPE_QCA5018:
		scn->targetdef = QCA5018_TARGETDEF;
		scn->target_ce_def = QCA5018_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCA5018");
		break;
#endif

#if defined(QCA5332_HEADERS_DEF)
	case TARGET_TYPE_QCA5332:
		scn->targetdef = QCA5332_TARGETDEF;
		scn->target_ce_def = QCA5332_CE_TARGETDEF;
		hif_info("TARGET_TYPE_QCA5332");
		break;
#endif

#if defined(QCA6390_HEADERS_DEF)
	case TARGET_TYPE_QCA6390:
		scn->targetdef = QCA6390_TARGETdef;
		scn->target_ce_def = QCA6390_CE_TARGETdef;
		hif_info("TARGET_TYPE_QCA6390");
		break;
#endif /* QCA6390_HEADERS_DEF */

#if defined(QCA6490_HEADERS_DEF)
	case TARGET_TYPE_QCA6490:
		scn->targetdef = QCA6490_TARGETdef;
		scn->target_ce_def = QCA6490_CE_TARGETdef;
		hif_info("TARGET_TYPE_QCA6490");
		break;
#endif /* QCA6490_HEADERS_DEF */

#if defined(KIWI_HEADERS_DEF)
	case TARGET_TYPE_KIWI:
		scn->targetdef = KIWI_TARGETdef;
		scn->target_ce_def = KIWI_CE_TARGETdef;
		hif_info("TARGET_TYPE_KIWI");
		break;

	case TARGET_TYPE_MANGO:
		scn->targetdef = KIWI_TARGETdef;
		scn->target_ce_def = KIWI_CE_TARGETdef;
		hif_info("TARGET_TYPE_MANGO");
		break;

	case TARGET_TYPE_PEACH:
		scn->targetdef = KIWI_TARGETdef;
		scn->target_ce_def = KIWI_CE_TARGETdef;
		hif_info("TARGET_TYPE_PEACH");
		break;
#endif /* KIWI_HEADERS_DEF */

#if defined(QCA6750_HEADERS_DEF)
	case TARGET_TYPE_QCA6750:
		scn->targetdef = QCA6750_TARGETdef;
		scn->target_ce_def = QCA6750_CE_TARGETdef;
		hif_info("TARGET_TYPE_QCA6750");
		break;
#endif /* QCA6750_HEADERS_DEF */

#if defined(WCN6450_HEADERS_DEF)
	case TARGET_TYPE_WCN6450:
		scn->targetdef = &wcn6450_targetdef;
		scn->target_ce_def = &wcn6450_ce_targetdef;
		hif_info("TARGET_TYPE_WCN6450");
		break;
#endif /* WCN6450_HEADERS_DEF */

#if defined(WCN7750_HEADERS_DEF)
	case TARGET_TYPE_WCN7750:
		scn->targetdef = WCN7750_TARGETdef;
		scn->target_ce_def = WCN7750_CE_TARGETdef;
		hif_info("TARGET_TYPE_WCN7750");
		break;
#endif /* WCN7750_HEADERS_DEF */

#if defined(QCC2072_HEADERS_DEF)
	case TARGET_TYPE_QCC2072:
		scn->targetdef = QCC2072_TARGETdef;
		scn->target_ce_def = QCC2072_CE_TARGETdef;
		hif_info("TARGET_TYPE_QCC2072");
		break;
#endif /* QCC2072_HEADERS_DEF */

	default:
		break;
	}
}

void hif_register_tbl_attach(struct hif_softc *scn, u32 hif_type)
{
	switch (hif_type) {
	case HIF_TYPE_AR6320V2:
		scn->hostdef = &ar6320v2_hostdef;
		break;
	case HIF_TYPE_ADRASTEA:
		scn->hostdef = &adrastea_hostdef;
		scn->host_shadow_regs = &adrastea_host_shadow_regs;
		break;
	case HIF_TYPE_QCN7605:
		scn->hostdef = &genoa_hostdef;
		scn->host_shadow_regs = &genoa_host_shadow_regs;
		break;
#if defined(AR6002_HEADERS_DEF)
	case HIF_TYPE_AR6002:
		scn->hostdef = AR6002_HOSTdef;
		break;
#endif
#if defined(AR6003_HEADERS_DEF)
	case HIF_TYPE_AR6003:
		scn->hostdef = AR6003_HOSTdef;
		break;
#endif
#if defined(AR6004_HEADERS_DEF)
	case HIF_TYPE_AR6004:
		scn->hostdef = AR6004_HOSTdef;
		break;
#endif
#if defined(AR9888_HEADERS_DEF)
	case HIF_TYPE_AR9888:
		scn->hostdef = AR9888_HOSTdef;
		break;
#endif
#if defined(AR9888V2_HEADERS_DEF)
	case HIF_TYPE_AR9888V2:
		scn->hostdef = AR9888V2_HOSTdef;
		break;
#endif
#if defined(AR900B_HEADERS_DEF)
	case HIF_TYPE_AR900B:
		scn->hostdef = AR900B_HOSTdef;
		break;
#endif
#if defined(QCA9984_HEADERS_DEF)
	case HIF_TYPE_QCA9984:
		scn->hostdef = QCA9984_HOSTdef;
		break;
#endif
#if defined(QCA9888_HEADERS_DEF)
	case HIF_TYPE_QCA9888:
		scn->hostdef = QCA9888_HOSTdef;
		break;
#endif

#if defined(QCA8074_HEADERS_DEF)
	case HIF_TYPE_QCA8074:
		scn->hostdef = QCA8074_HOSTdef;
		break;
#endif
#if defined(QCA8074V2_HEADERS_DEF)
	case HIF_TYPE_QCA8074V2:
		scn->hostdef = QCA8074V2_HOSTDEF;
		break;
#endif
#if defined(QCA6018_HEADERS_DEF)
	case HIF_TYPE_QCA6018:
		scn->hostdef = QCA6018_HOSTDEF;
		hif_info("HIF_TYPE_QCA6018");
		break;
#endif
#if defined(QCA6290_HEADERS_DEF)
	case HIF_TYPE_QCA6290:
		scn->hostdef = QCA6290_HOSTdef;
		break;
#endif
#if defined(QCN9000_HEADERS_DEF)
	case HIF_TYPE_QCN9000:
		scn->hostdef = QCN9000_HOSTDEF;
		break;
#endif
#if defined(QCN9224_HEADERS_DEF)
	case HIF_TYPE_QCN9224:
		if (scn->target_info.soc_version == 1)
			qdf_assert_always(0);
		scn->hostdef = QCN9224_HOSTDEF;
		break;
#endif
#if defined(QCN6122_HEADERS_DEF)
	case HIF_TYPE_QCN6122:
		scn->hostdef = QCN6122_HOSTDEF;
		break;
#endif
#if defined(QCN9160_HEADERS_DEF)
	case HIF_TYPE_QCN9160:
		scn->hostdef = QCN9160_HOSTDEF;
		break;
#endif
#if defined(QCN6432_HEADERS_DEF)
	case HIF_TYPE_QCN6432:
		scn->hostdef = QCN6432_HOSTDEF;
		break;
#endif
#if defined(QCA5424_HEADERS_DEF)
	case HIF_TYPE_QCA5424:
		scn->hostdef = QCA5424_HOSTDEF;
		break;
#endif
#if defined(QCA5018_HEADERS_DEF)
	case HIF_TYPE_QCA5018:
		scn->hostdef = QCA5018_HOSTDEF;
		break;
#endif
#if defined(QCA5332_HEADERS_DEF)
	case HIF_TYPE_QCA5332:
		scn->hostdef = QCA5332_HOSTDEF;
		break;
#endif
#if defined(QCA9574_HEADERS_DEF)
	case HIF_TYPE_QCA9574:
		scn->hostdef = QCA9574_HOSTDEF;
		hif_info("HIF_TYPE_QCA9574");
		break;
#endif

#if defined(QCA6390_HEADERS_DEF)
	case HIF_TYPE_QCA6390:
		scn->hostdef = QCA6390_HOSTdef;
		hif_info("HIF_TYPE_QCA6390");
		break;
#endif /* QCA6390_HEADERS_DEF */

#if defined(QCA6490_HEADERS_DEF)
	case HIF_TYPE_QCA6490:
		scn->hostdef = QCA6490_HOSTdef;
		hif_info("HIF_TYPE_QCA6490");
		break;
#endif /* QCA6490_HEADERS_DEF */

#if defined(KIWI_HEADERS_DEF)
	case HIF_TYPE_KIWI:
		scn->hostdef = KIWI_HOSTdef;
		hif_info("HIF_TYPE_KIWI");
		break;

	case HIF_TYPE_MANGO:
		scn->hostdef = KIWI_HOSTdef;
		hif_info("HIF_TYPE_MANGO");
		break;

	case HIF_TYPE_PEACH:
		scn->hostdef = KIWI_HOSTdef;
		hif_info("HIF_TYPE_PEACH");
		break;
#endif /* KIWI_HEADERS_DEF */

#if defined(QCA6750_HEADERS_DEF)
	case HIF_TYPE_QCA6750:
		scn->hostdef = QCA6750_HOSTdef;
		hif_info("HIF_TYPE_QCA6750");
		break;
#endif /* QCA6750_HEADERS_DEF */

#if defined(WCN6450_HEADERS_DEF)
	case HIF_TYPE_WCN6450:
		scn->hostdef = &wcn6450_hostdef;
		scn->host_shadow_regs = &wcn6450_host_shadow_regs;
		hif_info("HIF_TYPE_WCN6450");
		break;
#endif /* WCN6450_HEADERS_DEF */

#if defined(WCN7750_HEADERS_DEF)
	case HIF_TYPE_WCN7750:
		scn->hostdef = WCN7750_HOSTdef;
		hif_info("HIF_TYPE_WCN7750");
		break;
#endif /* WCN7750_HEADERS_DEF */

#if defined(QCC2072_HEADERS_DEF)
	case HIF_TYPE_QCC2072:
		scn->hostdef = QCC2072_HOSTdef;
		hif_info("HIF_TYPE_QCC2072");
		break;
#endif /* QCC2072_HEADERS_DEF */

	default:
		break;
	}
}
