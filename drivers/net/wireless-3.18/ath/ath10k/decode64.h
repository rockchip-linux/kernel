#ifndef _DECODE64_H_
#define _DECODE64_H_

int decode64(unsigned char *src, unsigned char *src_end,
	     unsigned char *dst);

int strip_nl(unsigned char *src, unsigned char *src_end,
	     unsigned char *dst);

#endif /* _DECODE64_H_ */
