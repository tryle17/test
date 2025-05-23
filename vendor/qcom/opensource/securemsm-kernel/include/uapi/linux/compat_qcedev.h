/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2014,2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _UAPI_COMPAT_QCEDEV__H
#define _UAPI_COMPAT_QCEDEV__H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/qcedev.h>
#include <asm-generic/posix_types.h>

/**
 * struct compat_buf_info - Buffer information
 * @offset:			Offset from the base address of the buffer
 *				(Used when buffer is allocated using PMEM)
 * @vaddr:			Virtual buffer address pointer
 * @len:				Size of the buffer
 */
struct compat_buf_info {
	union {
		__kernel_ulong_t	offset;
		__u32	vaddr;
	};
	__kernel_ulong_t	len;
};

/**
 * struct compat_qcedev_vbuf_info - Source and destination Buffer information
 * @src:				Array of buf_info for input/source
 * @dst:				Array of buf_info for output/destination
 */
struct compat_qcedev_vbuf_info {
	struct compat_buf_info	src[QCEDEV_MAX_BUFFERS];
	struct compat_buf_info	dst[QCEDEV_MAX_BUFFERS];
};

/**
 * struct compat_qcedev_pmem_info - Stores PMEM buffer information
 * @fd_src:			Handle to /dev/adsp_pmem used to allocate
 *				memory for input/src buffer
 * @src:				Array of buf_info for input/source
 * @fd_dst:			Handle to /dev/adsp_pmem used to allocate
 *				memory for output/dst buffer
 * @dst:				Array of buf_info for output/destination
 * @pmem_src_offset:		The offset from input/src buffer
 *				(allocated by PMEM)
 */
struct compat_qcedev_pmem_info {
	__s32		fd_src;
	struct compat_buf_info	src[QCEDEV_MAX_BUFFERS];
	__s32		fd_dst;
	struct compat_buf_info	dst[QCEDEV_MAX_BUFFERS];
};

/**
 * struct compat_qcedev_cipher_op_req - Holds the ciphering request information
 * @use_pmem (IN):	Flag to indicate if buffer source is PMEM
 *			QCEDEV_USE_PMEM/QCEDEV_NO_PMEM
 * @pmem (IN):		Stores PMEM buffer information.
 *			Refer struct qcedev_pmem_info
 * @vbuf (IN/OUT):	Stores Source and destination Buffer information
 *			Refer to struct qcedev_vbuf_info
 * @data_len (IN):	Total Length of input/src and output/dst in bytes
 * @in_place_op (IN):	Indicates whether the operation is inplace where
 *			source == destination
 *			When using PMEM allocated memory, must set this to 1
 * @enckey (IN):		128 bits of confidentiality key
 *			enckey[0] bit 127-120, enckey[1] bit 119-112,..
 *			enckey[15] bit 7-0
 * @encklen (IN):	Length of the encryption key(set to 128  bits/16
 *			bytes in the driver)
 * @iv (IN/OUT):		Initialization vector data
 *			This is updated by the driver, incremented by
 *			number of blocks encrypted/decrypted.
 * @ivlen (IN):		Length of the IV
 * @byteoffset (IN):	Offset in the Cipher BLOCK (applicable and to be set
 *			for AES-128 CTR mode only)
 * @alg (IN):		Type of ciphering algorithm: AES/DES/3DES
 * @mode (IN):		Mode use when using AES algorithm: ECB/CBC/CTR
 *			Applicable when using AES algorithm only
 * @op (IN):		Type of operation: QCEDEV_OPER_DEC/QCEDEV_OPER_ENC or
 *			QCEDEV_OPER_ENC_NO_KEY/QCEDEV_OPER_DEC_NO_KEY
 *
 * If use_pmem is set to 0, the driver assumes that memory was not allocated
 * via PMEM, and kernel will need to allocate memory and copy data from user
 * space buffer (data_src/dta_dst) and process accordingly and copy data back
 * to the user space buffer
 *
 * If use_pmem is set to 1, the driver assumes that memory was allocated via
 * PMEM.
 * The kernel driver will use the fd_src to determine the kernel virtual address
 * base that maps to the user space virtual address base for the  buffer
 * allocated in user space.
 * The final input/src and output/dst buffer pointer will be determined
 * by adding the offsets to the kernel virtual addr.
 *
 * If use of hardware key is supported in the target, user can configure the
 * key parameters (encklen, enckey) to use the hardware key.
 * In order to use the hardware key, set encklen to 0 and set the enckey
 * data array to 0.
 */
struct compat_qcedev_cipher_op_req {
	__u8					use_pmem;
	union {
		struct compat_qcedev_pmem_info	pmem;
		struct compat_qcedev_vbuf_info	vbuf;
	};
	__kernel_ulong_t				entries;
	__kernel_ulong_t				data_len;
	__u8							in_place_op;
	__u8							enckey[QCEDEV_MAX_KEY_SIZE];
	__kernel_ulong_t				encklen;
	__u8							iv[QCEDEV_MAX_IV_SIZE];
	__kernel_ulong_t				ivlen;
	__kernel_ulong_t				byteoffset;
	enum qcedev_cipher_alg_enum		alg;
	enum qcedev_cipher_mode_enum		mode;
	enum qcedev_oper_enum			op;
};

/**
 * struct qcedev_sha_op_req - Holds the hashing request information
 * @data (IN):			Array of pointers to the data to be hashed
 * @entries (IN):		Number of buf_info entries in the data array
 * @data_len (IN):		Length of data to be hashed
 * @digest (IN/OUT):		Returns the hashed data information
 * @diglen (OUT):		Size of the hashed/digest data
 * @authkey (IN):		Pointer to authentication key for HMAC
 * @authklen (IN):		Size of the authentication key
 * @alg (IN):			Secure Hash algorithm
 */
struct compat_qcedev_sha_op_req {
	struct compat_buf_info			data[QCEDEV_MAX_BUFFERS];
	__kernel_ulong_t				entries;
	__kernel_ulong_t				data_len;
	__u8							digest[QCEDEV_MAX_SHA_DIGEST];
	__kernel_ulong_t				diglen;
	__u32							authkey;
	__kernel_ulong_t				authklen;
	enum qcedev_sha_alg_enum		alg;
};

/**
 * struct compact_qcedev_map_buf_req - Holds the mapping request information
 * fd (IN):            Array of fds.
 * num_fds (IN):       Number of fds in fd[].
 * fd_size (IN):       Array of sizes corresponding to each fd in fd[].
 * fd_offset (IN):     Array of offset corresponding to each fd in fd[].
 * vaddr (OUT):        Array of mapped virtual address corresponding to
 *                     each fd in fd[].
 */
struct compat_qcedev_map_buf_req {
	__kernel_long_t	fd[QCEDEV_MAX_BUFFERS];
	__kernel_ulong_t	num_fds;
	__kernel_ulong_t	fd_size[QCEDEV_MAX_BUFFERS];
	__kernel_ulong_t	fd_offset[QCEDEV_MAX_BUFFERS];
	__u64      buf_vaddr[QCEDEV_MAX_BUFFERS];
};

/**
 * struct compat_qcedev_unmap_buf_req - Holds the hashing request information
 * fd (IN):	       Array of fds to unmap
 * num_fds (IN):       Number of fds in fd[].
 */
struct compat_qcedev_unmap_buf_req {
	__kernel_long_t	fd[QCEDEV_MAX_BUFFERS];
	__kernel_ulong_t	num_fds;
};

struct file;
long compat_qcedev_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg);

#define COMPAT_QCEDEV_IOCTL_ENC_REQ		\
	_IOWR(QCEDEV_IOC_MAGIC, 1, struct compat_qcedev_cipher_op_req)
#define COMPAT_QCEDEV_IOCTL_DEC_REQ		\
	_IOWR(QCEDEV_IOC_MAGIC, 2, struct compat_qcedev_cipher_op_req)
#define COMPAT_QCEDEV_IOCTL_SHA_INIT_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 3, struct compat_qcedev_sha_op_req)
#define COMPAT_QCEDEV_IOCTL_SHA_UPDATE_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 4, struct compat_qcedev_sha_op_req)
#define COMPAT_QCEDEV_IOCTL_SHA_FINAL_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 5, struct compat_qcedev_sha_op_req)
#define COMPAT_QCEDEV_IOCTL_GET_SHA_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 6, struct compat_qcedev_sha_op_req)
#define COMPAT_QCEDEV_IOCTL_LOCK_CE	\
	_IO(QCEDEV_IOC_MAGIC, 7)
#define COMPAT_QCEDEV_IOCTL_UNLOCK_CE	\
	_IO(QCEDEV_IOC_MAGIC, 8)
#define COMPAT_QCEDEV_IOCTL_GET_CMAC_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 9, struct compat_qcedev_sha_op_req)
#define COMPAT_QCEDEV_IOCTL_MAP_BUF_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 10, struct compat_qcedev_map_buf_req)
#define COMPAT_QCEDEV_IOCTL_UNMAP_BUF_REQ \
	_IOWR(QCEDEV_IOC_MAGIC, 11, struct compat_qcedev_unmap_buf_req)

long qcedev_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg);

#endif /* _UAPI_COMPAT_QCEDEV__H */
