#ifndef __QCOM_RPM_H__
#define __QCOM_RPM_H__

#include <linux/types.h>

struct qcom_rpm;

int qcom_rpm_write(struct qcom_rpm *rpm, int resource, u32 *buf, size_t count);

#endif
