// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2026 Kurt Skauen
 *
 * PadOS thread awareness using its versioned debugger interface.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/binarybuffer.h"
#include "helper/bits.h"
#include "helper/log.h"
#include "helper/types.h"
#include "server/gdb_server.h"
#include "target/armv7m.h"
#include "target/register.h"
#include "rtos.h"
#include "pados.h"

struct pados_state {
	uint32_t value;
	char name[PADOS_STATE_NAME_SIZE];
};

static int pados_read_info(struct rtos *rtos, uint32_t info[PADOS_INFO_COUNT])
{
	if (rtos->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	if (!rtos->symbols || !rtos->symbols[0].address)
		return ERROR_FAIL;

	uint8_t buffer[PADOS_INFO_COUNT * sizeof(uint32_t)];
	target_addr_t address = rtos->symbols[0].address;
	int retval = target_read_buffer(rtos->target, address, 16, buffer);
	if (retval != ERROR_OK)
		return retval;

	target_buffer_get_u32_array(rtos->target, buffer, 4, info);
	if (info[PADOS_VERSION] != PADOS_INTERFACE_VERSION ||
			info[PADOS_SIZE] != sizeof(buffer) ||
			info[PADOS_ARCHITECTURE] != PADOS_ARCH_CORTEX_M ||
			info[PADOS_POINTER_SIZE] != sizeof(uint32_t)) {
		LOG_ERROR("PadOS: unsupported debugger interface (version %" PRIu32 ", size %" PRIu32
			", architecture %" PRIu32 ", pointer size %" PRIu32 ")",
			info[PADOS_VERSION], info[PADOS_SIZE], info[PADOS_ARCHITECTURE], info[PADOS_POINTER_SIZE]);
		return ERROR_FAIL;
	}

	retval = target_read_buffer(rtos->target, address + 16, sizeof(buffer) - 16, buffer + 16);
	if (retval != ERROR_OK)
		return retval;
	target_buffer_get_u32_array(rtos->target, buffer, PADOS_INFO_COUNT, info);

	if (!info[PADOS_THREADS] || !info[PADOS_CURRENT_THREAD] ||
			!info[PADOS_THREAD_NAME_SIZE] || info[PADOS_THREAD_NAME_SIZE] > PADOS_MAX_NAME_SIZE ||
			!info[PADOS_THREAD_STATE_COUNT] || info[PADOS_THREAD_STATE_COUNT] > PADOS_MAX_STATES ||
			!info[PADOS_THREAD_STATES] || info[PADOS_CORE_REGISTER_COUNT] != ARMV7M_NUM_CORE_REGS ||
			info[PADOS_FLOATING_POINT_REGISTER_COUNT] != ARMV7M_D15 - ARMV7M_D0 + 2 ||
			!info[PADOS_FLOATING_POINT_REGISTER_OFFSETS] ||
			(info[PADOS_THREAD_STATE_SIZE] != 1 && info[PADOS_THREAD_STATE_SIZE] != 2 &&
			 info[PADOS_THREAD_STATE_SIZE] != 4)) {
		LOG_ERROR("PadOS: invalid debugger interface");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int pados_read_first_node(struct target *target, const uint32_t *info, uint32_t *node)
{
	return target_read_u32(target,
		(target_addr_t)info[PADOS_THREADS] + info[PADOS_THREAD_LIST_FIRST_OFFSET], node);
}

static int pados_read_states(struct target *target, const uint32_t *info, struct pados_state *states)
{
	for (uint32_t i = 0; i < info[PADOS_THREAD_STATE_COUNT]; i++) {
		uint8_t buffer[2 * sizeof(uint32_t)];
		int retval = target_read_buffer(target,
			(target_addr_t)info[PADOS_THREAD_STATES] + i * sizeof(buffer), sizeof(buffer), buffer);
		if (retval != ERROR_OK)
			return retval;
		states[i].value = target_buffer_get_u32(target, buffer);
		target_addr_t name = target_buffer_get_u32(target, buffer + sizeof(uint32_t));
		if (!name)
			return ERROR_FAIL;

		/* Read only through the terminator, even at the end of mapped memory. */
		for (size_t j = 0; j < sizeof(states[i].name); j++) {
			uint8_t character;
			retval = target_read_u8(target, name + j, &character);
			if (retval != ERROR_OK)
				return retval;
			states[i].name[j] = character;
			if (!character)
				break;
		}
		states[i].name[sizeof(states[i].name) - 1] = '\0';
	}
	return ERROR_OK;
}

static int pados_read_thread(struct target *target, const uint32_t *info,
		const struct pados_state *states, uint32_t address, struct thread_detail *thread)
{
	char name[PADOS_MAX_NAME_SIZE + 1];
	int retval = target_read_buffer(target, (target_addr_t)address + info[PADOS_THREAD_NAME_OFFSET],
		info[PADOS_THREAD_NAME_SIZE], (uint8_t *)name);
	if (retval != ERROR_OK)
		return retval;
	name[info[PADOS_THREAD_NAME_SIZE]] = '\0';

	uint32_t identifier;
	retval = target_read_u32(target, (target_addr_t)address + info[PADOS_THREAD_ID_OFFSET], &identifier);
	if (retval != ERROR_OK)
		return retval;
	if (identifier >= PADOS_IDLE_THREAD_ID) {
		LOG_ERROR("PadOS: invalid or reserved thread ID %" PRIu32, identifier);
		return ERROR_FAIL;
	}
	uint32_t priority;
	retval = target_read_u32(target, (target_addr_t)address + info[PADOS_THREAD_PRIORITY_OFFSET], &priority);
	if (retval != ERROR_OK)
		return retval;

	uint8_t state_buffer[sizeof(uint32_t)];
	retval = target_read_buffer(target, (target_addr_t)address + info[PADOS_THREAD_STATE_OFFSET],
		info[PADOS_THREAD_STATE_SIZE], state_buffer);
	if (retval != ERROR_OK)
		return retval;
	uint32_t state = state_buffer[0];
	if (info[PADOS_THREAD_STATE_SIZE] == 2)
		state = target_buffer_get_u16(target, state_buffer);
	else if (info[PADOS_THREAD_STATE_SIZE] == 4)
		state = target_buffer_get_u32(target, state_buffer);

	const char *state_name = "unknown";
	for (uint32_t i = 0; i < info[PADOS_THREAD_STATE_COUNT]; i++) {
		if (states[i].value == state) {
			state_name = states[i].name;
			break;
		}
	}

	/* GDB reserves zero for selecting any thread. */
	thread->threadid = identifier ? identifier : PADOS_IDLE_THREAD_ID;
	thread->exists = true;
	thread->thread_name_str = strdup(name[0] ? name : "unnamed");
	thread->extra_info_str = alloc_printf("ID: %" PRId32 ", State: %s, Priority: %" PRId64,
		(int32_t)identifier, state_name, (int64_t)(int32_t)priority + (int32_t)info[PADOS_PRIORITY_MINIMUM]);
	if (!thread->thread_name_str || !thread->extra_info_str)
		return ERROR_FAIL;
	return ERROR_OK;
}

static bool pados_thread_exists(const struct rtos *rtos, threadid_t thread_id)
{
	for (int i = 0; i < rtos->thread_count; i++) {
		if (rtos->thread_details[i].threadid == thread_id && rtos->thread_details[i].exists)
			return true;
	}
	return false;
}

static struct thread_detail *pados_append_thread(struct rtos *rtos, size_t *capacity)
{
	if (rtos->thread_count >= PADOS_MAX_THREADS) {
		LOG_ERROR("PadOS: thread list exceeds the safety limit");
		return NULL;
	}

	if ((size_t)rtos->thread_count == *capacity) {
		size_t new_capacity = *capacity ? *capacity * 2 : PADOS_INITIAL_THREAD_CAPACITY;
		if (new_capacity > PADOS_MAX_THREADS)
			new_capacity = PADOS_MAX_THREADS;
		struct thread_detail *threads = realloc(rtos->thread_details, new_capacity * sizeof(*threads));
		if (!threads) {
			LOG_ERROR("PadOS: out of memory for thread list");
			return NULL;
		}
		rtos->thread_details = threads;
		*capacity = new_capacity;
	}

	struct thread_detail *thread = &rtos->thread_details[rtos->thread_count++];
	/* A later read or string allocation can fail; keep cleanup safe. */
	memset(thread, 0, sizeof(*thread));
	return thread;
}

static int pados_update_threads(struct rtos *rtos)
{
	/* The caller ignores errors, so discard stale data before doing any reads. */
	rtos_free_threadlist(rtos);
	rtos->current_thread = 0;
	rtos->current_threadid = -1;

	uint32_t info[PADOS_INFO_COUNT];
	int retval = pados_read_info(rtos, info);
	if (retval != ERROR_OK)
		return retval;
	uint32_t node;
	retval = pados_read_first_node(rtos->target, info, &node);
	if (retval != ERROR_OK)
		return retval;
	uint32_t current;
	retval = target_read_u32(rtos->target, info[PADOS_CURRENT_THREAD], &current);
	if (retval != ERROR_OK)
		return retval;

	struct pados_state states[PADOS_MAX_STATES];
	retval = pados_read_states(rtos->target, info, states);
	if (retval != ERROR_OK)
		return retval;

	size_t capacity = 0;

	for (uint32_t i = 0; node && i < PADOS_MAX_THREADS; i++) {
		if (node <= info[PADOS_THREAD_NODE_OFFSET] || (node & 3)) {
			retval = ERROR_FAIL;
			goto fail;
		}
		uint32_t address = node - info[PADOS_THREAD_NODE_OFFSET];
		struct thread_detail *thread = pados_append_thread(rtos, &capacity);
		if (!thread) {
			retval = ERROR_FAIL;
			goto fail;
		}
		retval = pados_read_thread(rtos->target, info, states, address, thread);
		if (retval != ERROR_OK)
			goto fail;
		if (address == current)
			rtos->current_thread = thread->threadid;
		retval = target_read_u32(rtos->target, (target_addr_t)address + info[PADOS_THREAD_NEXT_OFFSET], &node);
		if (retval != ERROR_OK)
			goto fail;
	}
	if (node) {
		LOG_ERROR("PadOS: thread list exceeds the safety limit or contains a cycle");
		retval = ERROR_FAIL;
		goto fail;
	}

	if (!rtos->current_thread) {
		/* An existing idle entry rules out the unregistered bootstrap context. */
		if (pados_thread_exists(rtos, PADOS_IDLE_THREAD_ID)) {
			retval = ERROR_FAIL;
			goto fail;
		}
		struct thread_detail *thread = pados_append_thread(rtos, &capacity);
		if (!thread) {
			retval = ERROR_FAIL;
			goto fail;
		}
		thread->threadid = PADOS_IDLE_THREAD_ID;
		thread->exists = true;
		thread->thread_name_str = strdup("Current execution");
		if (!thread->thread_name_str) {
			retval = ERROR_FAIL;
			goto fail;
		}
		rtos->current_thread = thread->threadid;
	}
	return ERROR_OK;

fail:
	LOG_ERROR("PadOS: could not read a consistent thread list");
	rtos_free_threadlist(rtos);
	return retval;
}

static int pados_read_frame(struct target *target, const uint32_t *info, uint32_t stack,
		uint8_t frame[PADOS_MAX_FRAME_SIZE], uint32_t *frame_size, bool *extended)
{
	uint32_t basic_size = info[PADOS_BASIC_FRAME_SIZE];
	uint32_t extended_size = info[PADOS_EXTENDED_FRAME_SIZE];
	if (basic_size < sizeof(uint32_t) || basic_size > PADOS_MAX_FRAME_SIZE ||
			extended_size < basic_size || extended_size > PADOS_MAX_FRAME_SIZE ||
			(basic_size & 3) || (extended_size & 3) || (info[PADOS_EXCEPTION_RETURN_OFFSET] & 3) ||
			info[PADOS_EXCEPTION_RETURN_OFFSET] > basic_size - sizeof(uint32_t))
		return ERROR_FAIL;

	uint32_t exception_return;
	int retval = target_read_u32(target, (target_addr_t)stack + info[PADOS_EXCEPTION_RETURN_OFFSET],
		&exception_return);
	if (retval != ERROR_OK)
		return retval;

	if ((exception_return & 0xffffffe3) != 0xffffffe1)
		return ERROR_FAIL;

	/* EXC_RETURN bit 4 is clear when the floating-point context was saved. */
	*extended = !(exception_return & BIT(4));
	*frame_size = *extended ? extended_size : basic_size;
	return target_read_buffer(target, stack, *frame_size, frame);
}

static int pados_read_registers(struct target *target, const uint32_t *info, uint32_t stack,
		struct rtos_reg **reg_list, int *num_regs)
{
	uint8_t frame[PADOS_MAX_FRAME_SIZE];
	uint32_t frame_size;
	bool extended;
	int retval = pados_read_frame(target, info, stack, frame, &frame_size, &extended);
	if (retval != ERROR_OK)
		return retval;
	uint32_t table = info[extended ? PADOS_EXTENDED_REGISTER_OFFSETS : PADOS_BASIC_REGISTER_OFFSETS];
	if (!table)
		return ERROR_FAIL;
	uint8_t offset_buffer[ARMV7M_NUM_CORE_REGS * sizeof(uint16_t)];
	retval = target_read_buffer(target, table, sizeof(offset_buffer), offset_buffer);
	if (retval != ERROR_OK)
		return retval;
	uint16_t offsets[ARMV7M_NUM_CORE_REGS];
	target_buffer_get_u16_array(target, offset_buffer, ARRAY_SIZE(offsets), offsets);
	for (size_t i = 0; i < ARRAY_SIZE(offsets); i++) {
		if (i == ARMV7M_R13) {
			if (offsets[i] != PADOS_REGISTER_SP)
				return ERROR_FAIL;
		} else if (offsets[i] > frame_size - sizeof(uint32_t) || (offsets[i] & 3)) {
			return ERROR_FAIL;
		}
	}

	uint32_t xpsr = target_buffer_get_u32(target, frame + offsets[ARMV7M_XPSR]);
	target_addr_t restored_sp = (target_addr_t)stack + frame_size;
	if (xpsr & BIT(9))
		restored_sp += sizeof(uint32_t);
	if (restored_sp > UINT32_MAX)
		return ERROR_FAIL;

	struct rtos_reg *registers = calloc(ARMV7M_NUM_CORE_REGS, sizeof(*registers));
	if (!registers)
		return ERROR_FAIL;
	for (size_t i = 0; i < ARMV7M_NUM_CORE_REGS; i++) {
		registers[i].number = i;
		registers[i].size = 32;
		if (i == ARMV7M_R13)
			target_buffer_set_u32(target, registers[i].value, restored_sp);
		else
			memcpy(registers[i].value, frame + offsets[i], sizeof(uint32_t));
	}
	*reg_list = registers;
	*num_regs = ARMV7M_NUM_CORE_REGS;
	return ERROR_OK;
}

static int pados_find_thread(struct target *target, const uint32_t *info,
		uint32_t identifier, uint32_t *thread_address)
{
	uint32_t node;
	int retval = pados_read_first_node(target, info, &node);
	if (retval != ERROR_OK)
		return retval;

	*thread_address = 0;
	for (uint32_t i = 0; node && i < PADOS_MAX_THREADS; i++) {
		if (node <= info[PADOS_THREAD_NODE_OFFSET] || (node & 3))
			return ERROR_FAIL;
		uint32_t address = node - info[PADOS_THREAD_NODE_OFFSET];
		uint32_t candidate_identifier;
		retval = target_read_u32(target, (target_addr_t)address + info[PADOS_THREAD_ID_OFFSET],
			&candidate_identifier);
		if (retval != ERROR_OK)
			return retval;
		if (candidate_identifier == identifier) {
			*thread_address = address;
			return ERROR_OK;
		}
		retval = target_read_u32(target, (target_addr_t)address + info[PADOS_THREAD_NEXT_OFFSET], &node);
		if (retval != ERROR_OK)
			return retval;
	}
	return ERROR_FAIL;
}

static int pados_get_tls_address(struct rtos *rtos, uint32_t thread_id,
		uint32_t offset, uint32_t module, uint32_t *tls_address)
{
	if (!thread_id || thread_id > PADOS_IDLE_THREAD_ID || !pados_thread_exists(rtos, thread_id))
		return ERROR_FAIL;

	uint32_t info[PADOS_INFO_COUNT];
	int retval = pados_read_info(rtos, info);
	if (retval != ERROR_OK)
		return retval;

	uint32_t tls_offset;
	if (module == PADOS_TLS_MODULE_KERNEL)
		tls_offset = info[PADOS_THREAD_KERNEL_TLS_OFFSET];
	else if (module == PADOS_TLS_MODULE_USERSPACE)
		tls_offset = info[PADOS_THREAD_USERSPACE_TLS_OFFSET];
	else
		return ERROR_FAIL;
	if (tls_offset == UINT32_MAX)
		return ERROR_FAIL;

	uint32_t identifier = thread_id == PADOS_IDLE_THREAD_ID ? 0 : thread_id;
	uint32_t address;
	if (thread_id == rtos->current_thread) {
		/* The bootstrap idle thread need not be in the debugger list yet. */
		retval = target_read_u32(rtos->target, info[PADOS_CURRENT_THREAD], &address);
		if (retval != ERROR_OK)
			return retval;
		if (!address)
			return ERROR_FAIL;
		uint32_t current_identifier;
		retval = target_read_u32(rtos->target,
			(target_addr_t)address + info[PADOS_THREAD_ID_OFFSET], &current_identifier);
		if (retval != ERROR_OK)
			return retval;
		if (current_identifier != identifier)
			return ERROR_FAIL;
	} else {
		retval = pados_find_thread(rtos->target, info, identifier, &address);
		if (retval != ERROR_OK)
			return retval;
	}

	target_addr_t pointer_address = (target_addr_t)address + tls_offset;
	if (pointer_address > UINT32_MAX - sizeof(uint32_t) + 1)
		return ERROR_FAIL;
	uint32_t tls;
	retval = target_read_u32(rtos->target, pointer_address, &tls);
	if (retval != ERROR_OK)
		return retval;
	if (!tls)
		return ERROR_FAIL;

	target_addr_t result = (target_addr_t)tls + info[PADOS_TLS_DATA_OFFSET] + offset;
	if (result > UINT32_MAX)
		return ERROR_FAIL;
	*tls_address = result;
	return ERROR_OK;
}

/* Parse the thread ID, TLS offset, and module ID without accepting signs,
 * prefixes, trailing data, or values outside the target's 32-bit range.
 * OpenOCD's RTOS thread IDs do not use the multiprocess packet syntax. */
static bool pados_parse_tls_arguments(const char *cursor, const char *end, uint32_t fields[3])
{
	for (size_t i = 0; i < 3; i++) {
		const char *start = cursor;
		uint32_t value = 0;
		while (cursor < end && *cursor != ',') {
			char character = *cursor++;
			unsigned int digit;
			if (character >= '0' && character <= '9')
				digit = character - '0';
			else if (character >= 'a' && character <= 'f')
				digit = character - 'a' + 10;
			else if (character >= 'A' && character <= 'F')
				digit = character - 'A' + 10;
			else
				return false;
			if (value > (UINT32_MAX - digit) / 16)
				return false;
			value = value * 16 + digit;
		}
		if (cursor == start || (i < 2 ? cursor == end : cursor != end))
			return false;
		fields[i] = value;
		if (i < 2)
			cursor++;
	}
	return true;
}

static int pados_thread_packet(struct connection *connection, const char *packet, int packet_size)
{
	if (packet_size < 12 || memcmp(packet, "qGetTLSAddr:", 12))
		return rtos_thread_packet(connection, packet, packet_size);

	uint32_t fields[3];
	if (!pados_parse_tls_arguments(packet + 12, packet + packet_size, fields))
		return gdb_put_packet(connection, "E01", 3);

	struct target *target = get_target_from_connection(connection);
	uint32_t address;
	int retval = pados_get_tls_address(target->rtos, fields[0], fields[1], fields[2], &address);
	if (retval != ERROR_OK)
		return gdb_put_packet(connection, "E01", 3);

	char reply[2 * sizeof(address) + 1];
	int length = snprintf(reply, sizeof(reply), "%" PRIx32, address);
	return gdb_put_packet(connection, reply, length);
}

static int pados_get_thread_stack(struct rtos *rtos, threadid_t thread_id,
		uint32_t info[PADOS_INFO_COUNT], uint32_t *stack)
{
	/* The RTOS core obtains the current thread's registers from the processor. */
	if (thread_id <= 0 || thread_id > PADOS_IDLE_THREAD_ID ||
			thread_id == rtos->current_thread || !pados_thread_exists(rtos, thread_id))
		return ERROR_FAIL;

	int retval = pados_read_info(rtos, info);
	if (retval != ERROR_OK)
		return retval;
	uint32_t identifier = thread_id == PADOS_IDLE_THREAD_ID ? 0 : thread_id;
	uint32_t address;
	retval = pados_find_thread(rtos->target, info, identifier, &address);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(rtos->target,
		(target_addr_t)address + info[PADOS_THREAD_STACK_OFFSET], stack);
	if (retval != ERROR_OK)
		return retval;
	*stack &= info[PADOS_STACK_POINTER_MASK];
	if (!*stack || (*stack & 3))
		return ERROR_FAIL;
	return ERROR_OK;
}

static int pados_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	*reg_list = NULL;
	*num_regs = 0;
	uint32_t info[PADOS_INFO_COUNT];
	uint32_t stack;
	int retval = pados_get_thread_stack(rtos, thread_id, info, &stack);
	if (retval != ERROR_OK)
		return retval;
	return pados_read_registers(rtos->target, info, stack, reg_list, num_regs);
}

static int pados_get_thread_reg_value(struct rtos *rtos, threadid_t thread_id,
		uint32_t reg_num, uint32_t *size, uint8_t **value)
{
	*size = 0;
	*value = NULL;
	/* PadOS threads use PSP; MSP belongs to the shared exception stack. */
	if (reg_num == ARMV7M_MSP) {
		struct reg *reg = register_get_by_number(rtos->target->reg_cache, reg_num, true);
		if (!reg || !reg->exist || reg->size != 32)
			return ERROR_FAIL;
		if (!reg->valid) {
			int retval = reg->type->get(reg);
			if (retval != ERROR_OK)
				return retval;
		}
		*value = malloc(sizeof(uint32_t));
		if (!*value)
			return ERROR_FAIL;
		target_buffer_set_u32(rtos->target, *value, buf_get_u32(reg->value, 0, 32));
		*size = 32;
		return ERROR_OK;
	}
	if (reg_num < ARMV7M_NUM_CORE_REGS || reg_num == ARMV7M_PSP) {
		struct rtos_reg *registers;
		int count;
		int retval = pados_get_thread_reg_list(rtos, thread_id, &registers, &count);
		if (retval != ERROR_OK)
			return retval;
		*value = malloc(sizeof(uint32_t));
		if (*value) {
			uint32_t index = reg_num == ARMV7M_PSP ? ARMV7M_R13 : reg_num;
			memcpy(*value, registers[index].value, sizeof(uint32_t));
			*size = 32;
		}
		free(registers);
		return *value ? ERROR_OK : ERROR_FAIL;
	}

	/* An error prevents the GDB server from reading another thread's live registers. */
	if ((reg_num < ARMV7M_D0 || reg_num > ARMV7M_D15) && reg_num != ARMV7M_FPSCR)
		return ERROR_FAIL;

	uint32_t info[PADOS_INFO_COUNT];
	uint32_t stack;
	int retval = pados_get_thread_stack(rtos, thread_id, info, &stack);
	if (retval != ERROR_OK)
		return retval;
	uint8_t frame[PADOS_MAX_FRAME_SIZE];
	uint32_t frame_size;
	bool extended;
	retval = pados_read_frame(rtos->target, info, stack, frame, &frame_size, &extended);
	if (retval != ERROR_OK)
		return retval;
	if (!extended)
		return ERROR_FAIL;

	bool double_register = reg_num != ARMV7M_FPSCR;
	uint32_t index = double_register ? reg_num - ARMV7M_D0 :
		info[PADOS_FLOATING_POINT_REGISTER_COUNT] - 1;
	uint16_t offset;
	retval = target_read_u16(rtos->target,
		(target_addr_t)info[PADOS_FLOATING_POINT_REGISTER_OFFSETS] + index * sizeof(offset),
		&offset);
	if (retval != ERROR_OK)
		return retval;
	size_t bytes = double_register ? sizeof(uint64_t) : sizeof(uint32_t);
	if (frame_size < bytes || offset > frame_size - bytes || (offset & 3))
		return ERROR_FAIL;
	*value = malloc(bytes);
	if (!*value)
		return ERROR_FAIL;
	if (double_register) {
		uint64_t low = target_buffer_get_u32(rtos->target, frame + offset);
		uint64_t high = target_buffer_get_u32(rtos->target,
			frame + offset + sizeof(uint32_t));
		target_buffer_set_u64(rtos->target, *value, low | (high << 32));
	} else {
		memcpy(*value, frame + offset, bytes);
	}
	*size = bytes * 8;
	return ERROR_OK;
}

static int pados_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	*symbol_list = calloc(2, sizeof(**symbol_list));
	if (!*symbol_list)
		return ERROR_FAIL;
	(*symbol_list)[0].symbol_name = "_kernel_debugger_info";
	return ERROR_OK;
}

static bool pados_detect_rtos(struct target *target)
{
	return target->rtos->symbols && target->rtos->symbols[0].address;
}

static int pados_create(struct target *target)
{
	if (strcmp(target_type_name(target), "cortex_m") && strcmp(target_type_name(target), "hla_target")) {
		LOG_ERROR("PadOS: only Cortex-M targets are supported");
		return ERROR_FAIL;
	}
	target->rtos->gdb_thread_packet = pados_thread_packet;
	return ERROR_OK;
}

const struct rtos_type pados_rtos = {
	.name = "PadOS",
	.detect_rtos = pados_detect_rtos,
	.create = pados_create,
	.update_threads = pados_update_threads,
	.get_thread_reg_list = pados_get_thread_reg_list,
	.get_thread_reg_value = pados_get_thread_reg_value,
	.get_symbol_list_to_lookup = pados_get_symbol_list_to_lookup,
};
