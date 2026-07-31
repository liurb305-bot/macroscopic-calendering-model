# -*- coding: utf-8 -*-
"""建立膜厚20%压下、辊压后持续保载且不卸载的独立模型。"""

import create_selfsupport_yanshan_like_static_press as base


base.MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_20pct'
base.JOB_NAME = base.MODEL_NAME

# 0.150 mm × 20% = 0.030 mm；目标辊缝为0.120 mm。
base.CLAMP_DISPLACEMENT = -0.030
base.CLAMP_MAX_NUM_INC = 240
base.CLAMP_INITIAL_INC = 0.0125
base.CLAMP_MIN_INC = 1.0e-8
base.CLAMP_MAX_INC = 0.025

base.INCLUDE_UNLOAD_STEP = False
base.USE_UNLOAD_STABILIZATION = False
base.RELEASE_UPPER_CONTACT_IN_UNLOAD = False
base.USE_LOW_FRICTION_LOWER_SUPPORT_IN_UNLOAD = False

# 每5个增量保存一次场变量，用于比较10%和20%压下过程。
base.FIELD_OUTPUT_FREQUENCY = 5


if __name__ == '__main__':
    base.main()
