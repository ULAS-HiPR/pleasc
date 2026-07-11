#include <CAN/CAN_Frames.h>
#include <CAN/CAN_STM.h>
#include <stm32f0xx_hal.h>

#include <cstdint>

#ifndef PLEASC_FIRE_PULSE_MS
#define PLEASC_FIRE_PULSE_MS 250U
#endif

#ifndef PLEASC_ALLOW_FIRE_WITHOUT_CONTINUITY
#define PLEASC_ALLOW_FIRE_WITHOUT_CONTINUITY 0
#endif

// This board has no confirmed physical arm/RBF input. Keep outputs locked until
// the hardware interlock and command-authentication design are signed off.
#ifndef PLEASC_FIRE_ENABLED
#define PLEASC_FIRE_ENABLED 0
#endif

#ifndef REV1_ACCEPTED_RISK
#define REV1_ACCEPTED_RISK 0
#endif

#if PLEASC_FIRE_ENABLED && !REV1_ACCEPTED_RISK
#error "Pleasc firing requires explicit REV1_ACCEPTED_RISK=1"
#endif

CAN_HandleTypeDef hcan{};
IWDG_HandleTypeDef hiwdg{};

void Error_Handler();
void SystemClock_Config();
bool MX_CAN_Init();
void MX_GPIO_Init();

extern "C" {

struct OgmaBoardIdentity {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t struct_size;
    uint32_t board_id;
    uint32_t capabilities;
    uint32_t firmware_version;
    uint32_t firmware_build;
    uint32_t reserved0;
    uint32_t reserved1;
};

struct PleascStatus {
    uint32_t magic;
    uint32_t version;
    uint32_t uptime_ms;
    uint32_t loop_count;
    uint32_t clock_hz;
    uint32_t clock_source;
    uint32_t init_ok;
    uint32_t can_init_ok;
    uint32_t can_bus_off;
    uint32_t can_error;
    uint32_t can_tx_drops;
    uint32_t can_rx_count;
    uint32_t can_tx_count;
    uint32_t heartbeat_tx_count;
    uint32_t status_tx_count;
    uint32_t ack_tx_count;
    uint32_t armed_mask;
    uint32_t continuity_mask;
    uint32_t fire_pin_mask;
    uint32_t fault_latch;
    uint32_t last_fault;
    uint32_t last_channel;
    uint32_t fire_count;
    uint32_t rejected_count;
    uint32_t last_arm_ms;
    uint32_t last_fire_ms;
    uint32_t gpioa_idr;
    uint32_t gpiob_idr;
    uint32_t can_esr;
    uint32_t croi_last_seen_ms;
    uint32_t croi_timeout;
    uint32_t rejected_no_croi;
    uint32_t arm_expiry_count;
    uint32_t can_rx_overruns;
    uint32_t watchdog_refresh_count;
    uint32_t fire_enabled;
    uint32_t fired_mask;
    uint32_t last_command_sequence;
    uint32_t mission_tag;
    uint32_t croi_state;
    uint32_t rejected_auth;
    uint32_t rejected_replay;
    uint32_t rejected_state;
    uint32_t rejected_repeat;
    uint32_t rejected_mission;
    uint32_t arm_settle_rejects;
    uint32_t rev1_accepted_risk;
};

__attribute__((used)) volatile OgmaBoardIdentity ogma_board_identity{
    0x4F474944U,
    1U,
    sizeof(OgmaBoardIdentity),
    0x02U,
    0x41U,
    20260711U,
    0U,
    0U,
    0U,
};

__attribute__((used)) volatile PleascStatus pleasc_status{
    0x504C5343U,
    4U,
};
static_assert(sizeof(PleascStatus) == 188U, "PleascStatus wire contract changed");

}

namespace {

constexpr uint8_t kChannelCount = 4U;
constexpr uint32_t kStatusPeriodMs = 200U;
constexpr uint32_t kHeartbeatPeriodMs = 1000U;
constexpr uint32_t kBusRecoveryPeriodMs = 250U;
constexpr uint32_t kCroiTimeoutMs = 5000U;
constexpr uint32_t kArmLeaseMs = 2000U;
constexpr uint32_t kArmSettleMs = 100U;
constexpr uint8_t kTxQueueLen = 8U;
constexpr uint8_t kTxDrainBudget = 3U;

constexpr uint16_t kFirePins =
    GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
constexpr uint16_t kSeatPins =
    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
constexpr uint16_t kPyroEnablePin = GPIO_PIN_7;

enum FaultCode : uint8_t {
    FAULT_OK = 0U,
    FAULT_NO_CONTINUITY = 1U,
    FAULT_BAD_MAGIC = 2U,
    FAULT_NOT_ARMED = 3U,
    FAULT_BAD_CHANNEL = 4U,
    FAULT_BUSY = 5U,
    FAULT_CROI_TIMEOUT = 6U,
    FAULT_FIRE_LOCKED = 7U,
    FAULT_BAD_AUTH = 8U,
    FAULT_REPLAY = 9U,
    FAULT_BAD_STATE = 10U,
    FAULT_ALREADY_FIRED = 11U,
    FAULT_ARM_SETTLE = 12U,
    FAULT_MISSION_MISMATCH = 13U,
};

struct Pin {
    GPIO_TypeDef* port;
    uint16_t pin;
};

constexpr Pin kFireOutputs[kChannelCount] = {
    {GPIOB, GPIO_PIN_3},
    {GPIOB, GPIO_PIN_4},
    {GPIOB, GPIO_PIN_5},
    {GPIOB, GPIO_PIN_6},
};

constexpr Pin kSeatInputs[kChannelCount] = {
    {GPIOB, GPIO_PIN_10},
    {GPIOB, GPIO_PIN_11},
    {GPIOB, GPIO_PIN_12},
    {GPIOB, GPIO_PIN_13},
};

CAN_STM canbus(&hcan);

bool can_ready = false;
uint8_t armed_mask = 0U;
uint8_t fired_mask = 0U;
uint8_t fault_latch = 0U;
uint8_t last_fault = FAULT_OK;
uint8_t last_channel = 0xFFU;
uint32_t can_tx_drops = 0U;
uint32_t can_rx_count = 0U;
uint32_t can_tx_count = 0U;
uint32_t heartbeat_tx_count = 0U;
uint32_t status_tx_count = 0U;
uint32_t ack_tx_count = 0U;
uint32_t fire_count = 0U;
uint32_t rejected_count = 0U;
uint32_t rejected_no_croi = 0U;
uint32_t last_arm_ms = 0U;
uint32_t last_fire_ms = 0U;
uint32_t last_status_ms = 0U;
uint32_t last_heartbeat_ms = 0U;
uint32_t last_bus_recovery_ms = 0U;
uint32_t croi_last_seen_ms = 0U;
uint32_t arm_expiry_count = 0U;
uint32_t can_rx_overruns = 0U;
uint32_t watchdog_refresh_count = 0U;
uint16_t last_command_sequence = 0U;
uint16_t mission_tag = 0U;
uint8_t croi_state = 0U;
uint32_t rejected_auth = 0U;
uint32_t rejected_replay = 0U;
uint32_t rejected_state = 0U;
uint32_t rejected_repeat = 0U;
uint32_t rejected_mission = 0U;
uint32_t arm_settle_rejects = 0U;
uint32_t arm_started_ms[kChannelCount]{};

CAN_Frame tx_queue[kTxQueueLen]{};
uint8_t tx_head = 0U;
uint8_t tx_tail = 0U;
uint8_t tx_count = 0U;

bool fire_active = false;
uint8_t fire_channel = 0xFFU;
uint32_t fire_end_ms = 0U;

uint8_t bit_for_channel(uint8_t channel)
{
    return static_cast<uint8_t>(1U << channel);
}

bool valid_channel(uint8_t channel)
{
    return channel < kChannelCount;
}

void write_pyro_enable(bool enabled)
{
#if PLEASC_FIRE_ENABLED
    HAL_GPIO_WritePin(GPIOB, kPyroEnablePin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)enabled;
    HAL_GPIO_WritePin(GPIOB, kPyroEnablePin, GPIO_PIN_RESET);
#endif
}

void all_fire_outputs_low()
{
    HAL_GPIO_WritePin(GPIOB, kFirePins | kPyroEnablePin, GPIO_PIN_RESET);
}

void set_fire_output(uint8_t channel, bool enabled)
{
    if (!valid_channel(channel)) {
        return;
    }

    const Pin& output = kFireOutputs[channel];
    HAL_GPIO_WritePin(output.port, output.pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t read_continuity_mask()
{
    uint8_t mask = 0U;
    for (uint8_t channel = 0U; channel < kChannelCount; ++channel) {
        const Pin& input = kSeatInputs[channel];
        if (HAL_GPIO_ReadPin(input.port, input.pin) == GPIO_PIN_SET) {
            mask |= bit_for_channel(channel);
        }
    }
    return mask;
}

uint8_t read_fire_pin_mask()
{
    uint8_t mask = 0U;
    for (uint8_t channel = 0U; channel < kChannelCount; ++channel) {
        const Pin& output = kFireOutputs[channel];
        if (HAL_GPIO_ReadPin(output.port, output.pin) == GPIO_PIN_SET) {
            mask |= bit_for_channel(channel);
        }
    }
    return mask;
}

void record_fault(uint8_t channel, FaultCode fault)
{
    last_channel = channel;
    last_fault = fault;
    if (fault != FAULT_OK && fault < 8U) {
        fault_latch |= static_cast<uint8_t>(1U << fault);
        ++rejected_count;
    }
}

bool queue_tx_frame(const CAN_Frame& frame)
{
    if (tx_count >= kTxQueueLen) {
        tx_head = static_cast<uint8_t>((tx_head + 1U) % kTxQueueLen);
        --tx_count;
        ++can_tx_drops;
    }

    tx_queue[tx_tail] = frame;
    tx_tail = static_cast<uint8_t>((tx_tail + 1U) % kTxQueueLen);
    ++tx_count;
    return true;
}

bool send_frame(CAN_Frame& frame)
{
    if (!can_ready) {
        return false;
    }

    if (tx_count != 0U) {
        return queue_tx_frame(frame);
    }

    if (canbus.send(&frame)) {
        ++can_tx_count;
        return true;
    }

    return queue_tx_frame(frame);
}

void flush_tx_queue()
{
    if (!can_ready) {
        return;
    }

    for (uint8_t sent = 0U; sent < kTxDrainBudget && tx_count > 0U; ++sent) {
        CAN_Frame& frame = tx_queue[tx_head];
        if (!canbus.send(&frame)) {
            return;
        }

        tx_head = static_cast<uint8_t>((tx_head + 1U) % kTxQueueLen);
        --tx_count;
        ++can_tx_count;
    }
}

void send_ack(uint8_t channel, FaultCode fault,
              uint8_t command = 0U,
              uint16_t sequence = 0U,
              uint16_t command_mission_tag = 0U)
{
    const uint8_t result = fault != FAULT_OK
        ? static_cast<uint8_t>(PyroResult::FAULT)
        : pyro_is_fire_command(command)
            ? static_cast<uint8_t>(PyroResult::FIRED)
            : static_cast<uint8_t>(PyroResult::ACCEPTED);
    PYRO_ACK_Payload payload{
        channel,
        result,
        static_cast<uint8_t>(fault),
        command,
        sequence,
        command_mission_tag,
    };

    CAN_Frame frame = pack_frame(CAN_ID_PYRO_ACK, payload);
    if (send_frame(frame)) {
        ++ack_tx_count;
    }
}

void send_status()
{
    uint8_t faults = fault_latch;
    if (can_ready && canbus.is_bus_off()) {
        faults |= 0x40U;
    }
    if (can_ready && canbus.error() != 0U) {
        faults |= 0x80U;
    }

    PYRO_STATUS_Payload payload{
        armed_mask,
        read_continuity_mask(),
        faults,
        static_cast<uint8_t>(fire_active ? bit_for_channel(fire_channel) : 0U),
        fired_mask,
        croi_state,
        last_command_sequence,
    };

    CAN_Frame frame = pack_frame(CAN_ID_PYRO_STATUS, payload);
    if (send_frame(frame)) {
        ++status_tx_count;
    }
}

void send_heartbeat(uint32_t now_ms)
{
    uint8_t err = 0U;
    if (can_ready && canbus.is_bus_off()) {
        err |= CAN_HEARTBEAT_ERR_BUS_OFF;
    }
    if (can_ready && canbus.error() != 0U) {
        err |= CAN_HEARTBEAT_ERR_CAN_ERROR;
    }
    if (can_tx_drops != 0U) {
        err |= CAN_HEARTBEAT_ERR_TX_DROP;
    }
    if (croi_last_seen_ms == 0U || (now_ms - croi_last_seen_ms) >= kCroiTimeoutMs) {
        err |= CAN_HEARTBEAT_ERR_NODE_TIMEOUT;
    }

    const uint8_t state =
        fault_latch != 0U ? static_cast<uint8_t>(FlightState::FAULT) :
        armed_mask != 0U ? static_cast<uint8_t>(FlightState::ARMED) :
        static_cast<uint8_t>(FlightState::IDLE);

    HEARTBEAT_Payload payload{
        NODE_PLEASC,
        state,
        err,
        static_cast<uint8_t>((now_ms / 1000U) & 0xFFU),
    };

    CAN_Frame frame = pack_frame(CAN_ID_HEARTBEAT, payload);
    if (send_frame(frame)) {
        ++heartbeat_tx_count;
    }
}

bool croi_alive(uint32_t now_ms)
{
    return croi_last_seen_ms != 0U && (now_ms - croi_last_seen_ms) < kCroiTimeoutMs;
}

void disarm_all()
{
    armed_mask = 0U;
    fire_active = false;
    fire_channel = 0xFFU;
    all_fire_outputs_low();
}

void apply_arm_mask(uint8_t mask, uint32_t now_ms)
{
#if PLEASC_FIRE_ENABLED
    const uint8_t next_mask = static_cast<uint8_t>(mask & 0x0FU & ~fired_mask);
    const uint8_t newly_armed = static_cast<uint8_t>(next_mask & ~armed_mask);
    for (uint8_t channel = 0U; channel < kChannelCount; ++channel) {
        if ((newly_armed & bit_for_channel(channel)) != 0U) {
            arm_started_ms[channel] = now_ms;
        }
    }
    armed_mask = next_mask;
    write_pyro_enable(armed_mask != 0U || fire_active);
    last_arm_ms = now_ms;
#else
    (void)mask;
    (void)now_ms;
    disarm_all();
#endif
}

void reject_fire(uint8_t channel, FaultCode fault,
                 uint8_t command = 0U,
                 uint16_t sequence = 0U,
                 uint16_t command_mission_tag = 0U)
{
    record_fault(channel, fault);
    send_ack(channel, fault, command, sequence, command_mission_tag);
    send_status();
}

void start_fire(uint8_t channel, uint8_t command, uint32_t now_ms)
{
    fire_active = true;
    fire_channel = channel;
    fire_end_ms = now_ms + PLEASC_FIRE_PULSE_MS;
    armed_mask &= static_cast<uint8_t>(~bit_for_channel(channel));
    fired_mask |= bit_for_channel(channel);

    write_pyro_enable(true);
    set_fire_output(channel, true);

    last_channel = channel;
    last_fault = FAULT_OK;
    last_fire_ms = now_ms;
    ++fire_count;

    send_ack(channel, FAULT_OK, command,
             last_command_sequence, mission_tag);
    send_status();
}

void service_fire_pulse(uint32_t now_ms)
{
    if (!fire_active) {
        return;
    }

    if (static_cast<int32_t>(now_ms - fire_end_ms) < 0) {
        return;
    }

    set_fire_output(fire_channel, false);
    fire_active = false;
    fire_channel = 0xFFU;
    write_pyro_enable(armed_mask != 0U);
}

void service_arm_lease(uint32_t now_ms)
{
    if (armed_mask == 0U) {
        return;
    }
    if (!croi_alive(now_ms) || (now_ms - last_arm_ms) >= kArmLeaseMs) {
        disarm_all();
        ++arm_expiry_count;
        send_status();
    }
}

void handle_arm_frame(const CAN_Frame& frame, uint32_t now_ms)
{
    if (!croi_alive(now_ms)) {
        ++rejected_no_croi;
        record_fault(0xFFU, FAULT_CROI_TIMEOUT);
        send_ack(0xFFU, FAULT_CROI_TIMEOUT, PYRO_COMMAND_ARM);
        return;
    }

    PYRO_ARM_Payload payload{};
    if (!try_unpack_frame(frame, payload) || payload.command != PYRO_COMMAND_ARM) {
        record_fault(0xFFU, FAULT_BAD_MAGIC);
        send_ack(0xFFU, FAULT_BAD_MAGIC, PYRO_COMMAND_ARM);
        return;
    }

#if !PLEASC_FIRE_ENABLED
    (void)now_ms;
    record_fault(0xFFU, FAULT_FIRE_LOCKED);
    send_ack(0xFFU, FAULT_FIRE_LOCKED, payload.command,
             payload.sequence, payload.mission_tag);
    send_status();
    return;
#endif

    if (payload.command_tag != pyro_command_tag(
            payload.command, payload.channel_mask,
            payload.sequence, payload.mission_tag)) {
        ++rejected_auth;
        record_fault(0xFFU, FAULT_BAD_AUTH);
        send_ack(0xFFU, FAULT_BAD_AUTH, payload.command,
                 payload.sequence, payload.mission_tag);
        return;
    }
    if (!pyro_sequence_newer(payload.sequence, last_command_sequence)) {
        ++rejected_replay;
        record_fault(0xFFU, FAULT_REPLAY);
        send_ack(0xFFU, FAULT_REPLAY, payload.command,
                 payload.sequence, payload.mission_tag);
        return;
    }
    last_command_sequence = payload.sequence;
    if (payload.channel_mask != 0U &&
        (croi_state < static_cast<uint8_t>(FlightState::POWERED) || croi_state > 5U)) {
        ++rejected_state;
        record_fault(0xFFU, FAULT_BAD_STATE);
        send_ack(0xFFU, FAULT_BAD_STATE, payload.command,
                 payload.sequence, payload.mission_tag);
        return;
    }
    if (payload.channel_mask != 0U && mission_tag != 0U &&
        payload.mission_tag != mission_tag) {
        ++rejected_mission;
        record_fault(0xFFU, FAULT_MISSION_MISMATCH);
        send_ack(0xFFU, FAULT_MISSION_MISMATCH, payload.command,
                 payload.sequence, payload.mission_tag);
        return;
    }

    if (payload.channel_mask == 0U) {
        disarm_all();
    } else {
        if (mission_tag == 0U) {
            mission_tag = payload.mission_tag;
        }
        apply_arm_mask(payload.channel_mask, now_ms);
    }

    record_fault(0xFFU, FAULT_OK);
    send_ack(0xFFU, FAULT_OK, payload.command,
             payload.sequence, payload.mission_tag);
    send_status();
}

void handle_fire_frame(const CAN_Frame& frame, uint32_t now_ms)
{
    if (!croi_alive(now_ms)) {
        ++rejected_no_croi;
        reject_fire(0xFFU, FAULT_CROI_TIMEOUT);
        return;
    }

#if !PLEASC_FIRE_ENABLED
    const uint8_t locked_channel = frame.dlc > 0U ? frame.data[0] : 0xFFU;
    reject_fire(locked_channel, FAULT_FIRE_LOCKED);
    return;
#endif

    PYRO_FIRE_Payload payload{};
    if (!try_unpack_frame(frame, payload) || !pyro_is_fire_command(payload.command)) {
        reject_fire(0xFFU, FAULT_BAD_MAGIC);
        return;
    }

    const uint8_t channel = payload.channel;
    if (payload.command_tag != pyro_command_tag(
            payload.command, channel, payload.sequence, payload.mission_tag)) {
        ++rejected_auth;
        reject_fire(channel, FAULT_BAD_AUTH, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if (!pyro_sequence_newer(payload.sequence, last_command_sequence)) {
        ++rejected_replay;
        reject_fire(channel, FAULT_REPLAY, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    last_command_sequence = payload.sequence;
    if (mission_tag == 0U || payload.mission_tag != mission_tag) {
        ++rejected_mission;
        reject_fire(channel, FAULT_MISSION_MISMATCH, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if (croi_state != pyro_fire_expected_state(payload.command)) {
        ++rejected_state;
        reject_fire(channel, FAULT_BAD_STATE, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if (!valid_channel(channel)) {
        reject_fire(channel, FAULT_BAD_CHANNEL, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if (fire_active) {
        reject_fire(channel, FAULT_BUSY, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if ((fired_mask & bit_for_channel(channel)) != 0U) {
        ++rejected_repeat;
        reject_fire(channel, FAULT_ALREADY_FIRED, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if ((armed_mask & bit_for_channel(channel)) == 0U) {
        reject_fire(channel, FAULT_NOT_ARMED, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
    if ((now_ms - arm_started_ms[channel]) < kArmSettleMs) {
        ++arm_settle_rejects;
        reject_fire(channel, FAULT_ARM_SETTLE, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
#if !PLEASC_ALLOW_FIRE_WITHOUT_CONTINUITY
    if ((read_continuity_mask() & bit_for_channel(channel)) == 0U) {
        reject_fire(channel, FAULT_NO_CONTINUITY, payload.command,
                    payload.sequence, payload.mission_tag);
        return;
    }
#endif

    start_fire(channel, payload.command, now_ms);
}

void process_rx_frame(const CAN_Frame& frame, uint32_t now_ms)
{
    ++can_rx_count;

    if (CAN_ID_IS_HEARTBEAT(frame.id)) {
        HEARTBEAT_Payload payload{};
        if (try_unpack_frame(frame, payload) && payload.node_id == NODE_CROI) {
            croi_last_seen_ms = now_ms;
            croi_state = payload.state;
        }
        return;
    }

    if (frame.id == CAN_ID_FLIGHT_STATE && croi_alive(now_ms)) {
        FLIGHT_STATE_Payload payload{};
        if (try_unpack_frame(frame, payload)) {
            croi_state = payload.state;
        }
        return;
    }

    switch (frame.id) {
    case CAN_ID_PYRO_ARM:
        handle_arm_frame(frame, now_ms);
        break;
    case CAN_ID_PYRO_FIRE:
        handle_fire_frame(frame, now_ms);
        break;
    case CAN_ID_CONFIG_CMD:
        if (frame.dlc > 0U && frame.data[0] == 0U && croi_alive(now_ms)) {
            disarm_all();
            send_status();
        } else if (frame.dlc > 0U && frame.data[0] == 0U) {
            ++rejected_no_croi;
            record_fault(0xFFU, FAULT_CROI_TIMEOUT);
        }
        break;
    default:
        break;
    }
}

void service_can_rx(uint32_t now_ms)
{
    if (!can_ready) {
        return;
    }

    if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) {
        __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV0);
        ++can_rx_overruns;
    }

    CAN_Frame frame{};
    while (canbus.receive(&frame)) {
        process_rx_frame(frame, now_ms);
    }
}

void service_bus_health(uint32_t now_ms)
{
    if (!can_ready || !canbus.is_bus_off()) {
        return;
    }
    if ((now_ms - last_bus_recovery_ms) < kBusRecoveryPeriodMs) {
        return;
    }

    (void)canbus.recover_from_bus_off();
    last_bus_recovery_ms = now_ms;
}

void update_status(uint32_t now_ms)
{
    pleasc_status.uptime_ms = now_ms;
    pleasc_status.loop_count++;
    pleasc_status.clock_hz = SystemCoreClock;
    pleasc_status.init_ok = can_ready ? 1U : 0U;
    pleasc_status.can_init_ok = can_ready ? 1U : 0U;
    pleasc_status.can_bus_off = can_ready && canbus.is_bus_off() ? 1U : 0U;
    pleasc_status.can_error = can_ready ? canbus.error() : 0U;
    pleasc_status.can_tx_drops = can_tx_drops;
    pleasc_status.can_rx_count = can_rx_count;
    pleasc_status.can_tx_count = can_tx_count;
    pleasc_status.heartbeat_tx_count = heartbeat_tx_count;
    pleasc_status.status_tx_count = status_tx_count;
    pleasc_status.ack_tx_count = ack_tx_count;
    pleasc_status.armed_mask = armed_mask;
    pleasc_status.continuity_mask = read_continuity_mask();
    pleasc_status.fire_pin_mask = read_fire_pin_mask();
    pleasc_status.fault_latch = fault_latch;
    pleasc_status.last_fault = last_fault;
    pleasc_status.last_channel = last_channel;
    pleasc_status.fire_count = fire_count;
    pleasc_status.rejected_count = rejected_count;
    pleasc_status.last_arm_ms = last_arm_ms;
    pleasc_status.last_fire_ms = last_fire_ms;
    pleasc_status.gpioa_idr = GPIOA->IDR;
    pleasc_status.gpiob_idr = GPIOB->IDR;
    pleasc_status.can_esr = hcan.Instance != nullptr ? hcan.Instance->ESR : 0U;
    pleasc_status.croi_last_seen_ms = croi_last_seen_ms;
    pleasc_status.croi_timeout = croi_alive(now_ms) ? 0U : 1U;
    pleasc_status.rejected_no_croi = rejected_no_croi;
    pleasc_status.arm_expiry_count = arm_expiry_count;
    pleasc_status.can_rx_overruns = can_rx_overruns;
    pleasc_status.watchdog_refresh_count = watchdog_refresh_count;
    pleasc_status.fire_enabled = PLEASC_FIRE_ENABLED ? 1U : 0U;
    pleasc_status.fired_mask = fired_mask;
    pleasc_status.last_command_sequence = last_command_sequence;
    pleasc_status.mission_tag = mission_tag;
    pleasc_status.croi_state = croi_state;
    pleasc_status.rejected_auth = rejected_auth;
    pleasc_status.rejected_replay = rejected_replay;
    pleasc_status.rejected_state = rejected_state;
    pleasc_status.rejected_repeat = rejected_repeat;
    pleasc_status.rejected_mission = rejected_mission;
    pleasc_status.arm_settle_rejects = arm_settle_rejects;
    pleasc_status.rev1_accepted_risk = REV1_ACCEPTED_RISK ? 1U : 0U;
}

bool init_watchdog()
{
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1249U;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    return HAL_IWDG_Init(&hiwdg) == HAL_OK;
}

bool configure_hse_48mhz()
{
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PREDIV = RCC_PREDIV_DIV1;
    osc.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    RCC_ClkInitTypeDef clk{};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }

    SystemCoreClockUpdate();
    pleasc_status.clock_source = 1U;
    return true;
}

bool configure_hsi48()
{
#if defined(RCC_OSCILLATORTYPE_HSI48)
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    RCC_ClkInitTypeDef clk{};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }

    SystemCoreClockUpdate();
    pleasc_status.clock_source = 2U;
    return true;
#else
    return false;
#endif
}

} // namespace

int main()
{
    (void)ogma_board_identity.magic;
    (void)pleasc_status.magic;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    all_fire_outputs_low();
    if (!init_watchdog()) {
        Error_Handler();
    }

    const bool can_hw_ok = MX_CAN_Init();
    can_ready = can_hw_ok && canbus.init();

    last_status_ms = HAL_GetTick();
    last_heartbeat_ms = last_status_ms;
    last_bus_recovery_ms = last_status_ms;
    update_status(last_status_ms);
    send_status();
    send_heartbeat(last_status_ms);

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        service_bus_health(now_ms);
        flush_tx_queue();
        service_can_rx(now_ms);
        service_fire_pulse(now_ms);
        service_arm_lease(now_ms);

        if ((now_ms - last_status_ms) >= kStatusPeriodMs) {
            send_status();
            last_status_ms = now_ms;
        }
        if ((now_ms - last_heartbeat_ms) >= kHeartbeatPeriodMs) {
            send_heartbeat(now_ms);
            last_heartbeat_ms = now_ms;
        }

        update_status(now_ms);
        if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) {
            ++watchdog_refresh_count;
        }
        HAL_Delay(10U);
    }
}

void SystemClock_Config()
{
    if (configure_hse_48mhz() || configure_hsi48()) {
        return;
    }

    Error_Handler();
}

bool MX_CAN_Init()
{
    hcan.Instance = CAN;
    hcan.Init.Prescaler = 6U;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    return HAL_CAN_Init(&hcan) == HAL_OK;
}

void MX_GPIO_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, kFirePins | kPyroEnablePin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio{};
    gpio.Pin = kFirePins | kPyroEnablePin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio = {};
    gpio.Pin = kSeatPins;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_MspInit()
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* handle)
{
    if (handle->Instance != CAN) {
        return;
    }

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* handle)
{
    if (handle->Instance != CAN) {
        return;
    }

    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}

void Error_Handler()
{
    all_fire_outputs_low();
    __disable_irq();
    while (true) {
        // IWDG resets the MCU; GPIO initialization drives every fire output low.
    }
}
