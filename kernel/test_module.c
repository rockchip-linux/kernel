/*
 * "hello world" kernel module
 */

#define pr_fmt(fmt) "test_module: " fmt

#include <linux/module.h>

static int __init test_module_init(void)
{
	pr_info("Hello, world\n");

	return 0;
}

module_init(test_module_init);

static void __exit test_module_exit(void)
{
	pr_info("Goodbye\n");
}

module_exit(test_module_exit);

MODULE_AUTHOR("Kees Cook <keescook@chromium.org>");
MODULE_LICENSE("GPL");
