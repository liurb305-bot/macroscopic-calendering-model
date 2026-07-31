# -*- coding: utf-8 -*-
"""建立辊压后保持压下、不执行卸载的独立对照模型。"""

import create_selfsupport_yanshan_like_static_press as base


# 独立模型与作业名，不覆盖含卸载步骤的原模型。
base.MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload'
base.JOB_NAME = base.MODEL_NAME

# 仅保留 Clamp_Down 和 Hold；上下辊-膜接触在 Hold 中持续有效。
base.INCLUDE_UNLOAD_STEP = False
base.USE_UNLOAD_STABILIZATION = False
base.RELEASE_UPPER_CONTACT_IN_UNLOAD = False
base.USE_LOW_FRICTION_LOWER_SUPPORT_IN_UNLOAD = False

# 每5个增量保存一次场变量，便于查看压下过程中的应力应变演化。
base.FIELD_OUTPUT_FREQUENCY = 5


if __name__ == '__main__':
    base.main()
