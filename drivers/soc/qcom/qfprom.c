#include <linux/err.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/slab.h>

#define QFPROM_MAX_ARGS	2

static char *__qfprom_get_data(struct device *dev,
				bool devm, int idx, int *len)
{
	struct device_node *syscon_np, *np = dev->of_node;
	struct regmap *rm;
	struct of_phandle_args args;
	int rc, stride = 4;
	u32 offset, size;
	char *data;

	if (!np)
		return ERR_PTR(-EINVAL);

	rc = of_parse_phandle_with_fixed_args(np, "qcom,qfprom",
					      QFPROM_MAX_ARGS, idx, &args);
	if (rc)
		return ERR_PTR(rc);

	syscon_np = args.np;

	of_property_read_u32(syscon_np, "stride", &stride);

	if (stride > 4)
		return ERR_PTR(-EINVAL);

	if (args.args_count < QFPROM_MAX_ARGS) {
		dev_err(dev, "Insufficient qfprom arguments %d\n",
			args.args_count);
		return ERR_PTR(-EINVAL);
	}

	rm = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(rm))
		return ERR_CAST(rm);

	offset = args.args[0];
	size = args.args[1];

	of_node_put(syscon_np);

	if (devm)
		data = devm_kzalloc(dev, size, GFP_KERNEL | GFP_ATOMIC);
	else
		data = kzalloc(size, GFP_KERNEL | GFP_ATOMIC);

	if (!data)
		return ERR_PTR(-ENOMEM);

	rc = regmap_bulk_read(rm, offset, data, size/stride);
	if (rc < 0) {
		if (devm)
			devm_kfree(dev, data);
		else
			kfree(data);

		return ERR_PTR(rc);
	}

	*len = size;

	return data;
}

static char *__qfprom_get_data_byname(struct device *dev,
				       bool devm, const char *name, int *len)
{
	int index = 0;

	if (name)
		index = of_property_match_string(dev->of_node,
						 "qcom,qfprom-names", name);

	return __qfprom_get_data(dev, devm, index, len);
}

char *devm_qfprom_get_data_byname(struct device *dev,
					  const char *name, int *len)
{
	return __qfprom_get_data_byname(dev, true, name, len);
}
EXPORT_SYMBOL_GPL(devm_qfprom_get_data_byname);

char *devm_qfprom_get_data(struct device *dev,
				   int index, int *len)
{
	return __qfprom_get_data(dev, true, index, len);
}
EXPORT_SYMBOL_GPL(devm_qfprom_get_data);

/**
 * qfprom_get_data_byname(): Reads qfprom data by name
 *
 * @dev: device which is requesting qfprom data
 * @index: name of qfprom resources specified "qcom,qfprom-names" DT property.
 * @len: length of data read from qfprom.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a data buffer. The buffer should be freed by the user once its
 * finished working with it kfree.
 **/
char *qfprom_get_data_byname(struct device *dev,
				     const char *name, int *len)
{
	return __qfprom_get_data_byname(dev, false, name, len);
}
EXPORT_SYMBOL_GPL(qfprom_get_data_byname);

/**
 * qfprom_get_data(): Reads qfprom data from the index
 *
 * @dev: device which is requesting qfprom data
 * @index: index into qfprom resources specified "qcom,qfprom" DT property.
 * @len: length of data read from qfprom.
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a data buffer. The buffer should be freed by the user once its
 * finished working with it kfree.
 **/
char *qfprom_get_data(struct device *dev,
				   int index, int *len)
{
	return __qfprom_get_data(dev, false, index, len);
}
EXPORT_SYMBOL_GPL(qfprom_get_data);
