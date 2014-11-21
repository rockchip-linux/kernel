extern void __init bootmem_init(void);
extern void __init arm64_swiotlb_init(void);

void fixup_init(void);
void adjust_exec_mem(unsigned long start, unsigned long end);

