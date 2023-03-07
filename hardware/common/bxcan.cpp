/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2022 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <cstdint>
#include <cstring>
#include "hardware_interface.hpp"
#include "hal.h"
#include "settings.hpp"


/// This is defined by the bxCAN hardware.
/// Devices with only one CAN interface have 14 filters (e.g. F103).
/// Devices with two CAN interfaces (e.g. F105, F446) have 28 filters, which are shared equally.
constexpr std::uint8_t NUM_ACCEPTANCE_FILTERS  = 14;

///< CAN Driver instances if redundant. Instance 0 is always master CAN
extern CANDriver* pcan[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

/**
 * CAN Configuration, will be applied to all instances of the CAN Driver
 *
 * automatic wakeup, automatic recover from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  0
};

/// Bit timing parameters. Use bxCANComputeTimings() to derive these from the desired bus data rate.
/// Some applications may prefer to store pre-computed parameters instead of calculating them at runtime.
typedef struct bxcan_timings_s
{
    uint16_t bit_rate_prescaler;     /// [1, 1024]
    uint8_t  bit_segment_1;          /// [1, 16]
    uint8_t  bit_segment_2;          /// [1, 8]
    uint8_t  max_resync_jump_width;  /// [1, 4] (recommended value is 1)
} bxcan_timings_ts;

///< currently set bit rate
std::uint32_t cur_bitrate = 125000;

///< Eventsources for the CAN driver instances
struct hardware::can::events_s hardware::can::event[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

/**
 * @fn bool bxCANComputeTimings(const uint32_t, const uint32_t, BxCANTimings* const)
 * @brief calculate the bit rate register value for a given bitrate
 *
 * Given the bxCAN macrocell clock rate and the desired bit rate, compute the optimal timing register configuration.
 * The solution is optimized per the recommendations given in the specifications of DS-015, DeviceNet, CANOpen.
 * Units are SI. Typically, CAN is clocked from PCLK1.
 *
 * @param peripheral_clock_rate
 * @param target_bitrate
 * @param out_timings
 * @return Returns true on success. Returns false if the requested bit rate cannot be set up at the current clock rate.
 *
 * Function taken from bxcan driver of UAVCAN.
 */
static bool hardware_can_ComputeTimings(const uint32_t      peripheral_clock_rate,  //
                         	 	 	 	const uint32_t      target_bitrate,         //
										bxcan_timings_ts* const out_timings)
{
    if (target_bitrate < 1000U)
    {
        return false;
    }

    osalDbgAssert(out_timings != NULL, "out timings are NULL");

    // Clang-Tidy raises an error recommending the use of memset_s() instead.
    // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
    memset(out_timings, 0, sizeof(*out_timings));  // NOLINT

    // Hardware configuration
    static const uint8_t MaxBS1 = 16U;
    static const uint8_t MaxBS2 = 8U;

    // Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    //      CAN in Automation, 2003
    //
    // According to the source, optimal quanta per bit are:
    //   Bitrate        Optimal Maximum
    //   1000 kbps      8       10
    //   500  kbps      16      17
    //   250  kbps      16      17
    //   125  kbps      16      17
    const uint8_t max_quanta_per_bit = (uint8_t)((target_bitrate >= 1000000) ? 10U : 17U);
    osalDbgAssert(max_quanta_per_bit <= (MaxBS1 + MaxBS2), "No valid solution for target bit rate");

    static const uint16_t MaxSamplePointLocationPermill = 900U;

    // Computing (prescaler * BS):
    //   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    //   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    // let:
    //   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    //   PRESCALER_BS = PRESCALER * BS
    // ==>
    //   PRESCALER_BS = PCLK / BITRATE
    const uint32_t prescaler_bs = peripheral_clock_rate / target_bitrate;

    // Searching for such prescaler value so that the number of quanta per bit is highest.
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1U);

    while ((prescaler_bs % (1U + bs1_bs2_sum)) != 0U)
    {
        if (bs1_bs2_sum <= 2U)
        {
            return false;  // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1U + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
        return false;  // No solution
    }

    // Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    // We need to find such values so that the sample point is as close as possible to the optimal value,
    // which is 87.5%, which is 7/8.
    //
    //   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    //   {{bs2 -> (1 + bs1)/7}}
    //
    // Hence:
    //   bs2 = (1 + bs1) / 7
    //   bs1 = (7 * bs1_bs2_sum - 1) / 8
    //
    // Sample point location can be computed as follows:
    //   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    //
    // Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    //   - With rounding to nearest
    //   - With rounding to zero
    uint8_t bs1 = (uint8_t)(((7U * bs1_bs2_sum - 1U) + 4U) / 8U);  // Trying rounding to nearest first
    uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
    osalDbgAssert(bs1_bs2_sum > bs1, "No sample point found");

    {
        const uint16_t sample_point_permill = (uint16_t)(1000U * (1U + bs1) / (1U + bs1 + bs2));

        if (sample_point_permill > MaxSamplePointLocationPermill)  // Strictly more!
        {
            bs1 = (uint8_t)((7U * bs1_bs2_sum - 1U) / 8U);  // Nope, too far; now rounding to zero
            bs2 = (uint8_t)(bs1_bs2_sum - bs1);
        }
    }

    const bool valid = (bs1 >= 1U) && (bs1 <= MaxBS1) && (bs2 >= 1U) && (bs2 <= MaxBS2);

    // Final validation
    // Helpful Python:
    // def sample_point_from_btr(x):
    //     assert 0b0011110010000000111111000000000 & x == 0
    //     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
    //     return (1+ts1+1)/(1+ts1+1+ts2+1)
    if ((target_bitrate != (peripheral_clock_rate / (prescaler * (1U + bs1 + bs2)))) || !valid)
    {
        // This actually means that the algorithm has a logic error, hence assert(0).
    	osalDbgAssert(0, "Algorithm has a logic error");
        return false;
    }

    out_timings->bit_rate_prescaler    = (uint16_t) prescaler;
    out_timings->max_resync_jump_width = 1U;  // One is recommended by DS-015, CANOpen, and DeviceNet
    out_timings->bit_segment_1         = bs1;
    out_timings->bit_segment_2         = bs2;

    return true;
}

/**
 * @fn void hardware_can_Init(void)
 * @brief internal can bus initialization function
 *
 */
void hardware_can_Init(void)
{
	const std::uint32_t FALLBACK_BITRATE = 125000;

	cur_bitrate = settings.cyphal.nbitrate;

	// if successful SetBitrate will start the can interfaces.
	while(!hardware::can::SetBitrate(cur_bitrate, 0, false))
	{
		// if desired bit rate is not possible then fall back to a standard bit rate
		if(cur_bitrate != FALLBACK_BITRATE)
		{
			cur_bitrate = FALLBACK_BITRATE;
		}
		else
		{
			// this should not happen
			osalSysHalt("no Bit Rate set.");
			break;
		}
	}
}


/**
 * @fn bool Transmit(const std::uint_fast8_t, CanardFrame&)
 * @brief transmit frame via selected interface.
 *
 * @param interface 	index of the CAN interface used to transmit the frame.
 * @param frame			uavcan:can frame to be transmitted.
 * @return				true on error.
 */
bool hardware::can::Transmit(const std::uint_fast8_t interface, const CanardFrame& frame)
{
	CANTxFrame txmsg;

	txmsg.DLC = frame.payload_size;
	txmsg.EID = frame.extended_can_id;
	txmsg.IDE = 1;

	std::memcpy(txmsg.data8, frame.payload, frame.payload_size);

	msg_t result = canTransmit(pcan[interface], CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE);

	return (result != MSG_OK);
}

/**
 * @fn bool Receive(const std::uint_fast8_t, CanardFrame&)
 * @brief receive frame from selected interface.
 *
 * @param interface		index of the CAN interface used to transmit the frame.
 * @param frame			uavcan:can frame recived.
 * @return				true on success.
 */
bool hardware::can::Receive(const std::uint_fast8_t interface, CanardFrame& frame)
{
	static CANRxFrame rxmsg;

	msg_t result = canReceive(pcan[interface], CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

	// only extended id frames are valid frames
	if(!rxmsg.IDE)
	{
		result = !MSG_OK;
	}
	else
	{
		frame.payload_size = rxmsg.DLC;
		frame.extended_can_id = rxmsg.EID;
		frame.payload = rxmsg.data8;
	}

	return (result == MSG_OK);
}

/**
 * @fn bool SetBitrate(const std::uint32_t)
 * @brief set the desired bitrate for the can bus.
 *
 * @param nbitrate 		desired nominal bitrate in bits/second
 * @param dbitrate 		desired data bitrate in bits/second (only FD Mode)
 * @param fd_mode		True enables FD Mode
 * @return				True on success.
 */
bool hardware::can::SetBitrate(const std::uint32_t nbitrate, const std::uint32_t dbitrate, const bool fd_mode)
{
	bool result = false;
	bxcan_timings_ts timings;
	(void) dbitrate;

	if (fd_mode) return result;

	if(			nbitrate >= 125000
			&& 	nbitrate <= 1000000)
	{
		result = hardware_can_ComputeTimings(STM32_PCLK1, nbitrate, &timings);

		if(result)
		{
			cancfg.btr = 	  CAN_BTR_SJW(timings.max_resync_jump_width - 1)
							| CAN_BTR_TS1(timings.bit_segment_1 - 1)
							| CAN_BTR_TS2(timings.bit_segment_2 - 1)
							| CAN_BTR_BRP(timings.bit_rate_prescaler - 1);

			cur_bitrate = nbitrate;
			for(std::uint_fast8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
			{
				canStop(pcan[i]);
				osalThreadSleepMilliseconds(10);
				canStart(pcan[i], &cancfg);
			}
		}
	}

	return result;
}

/**
 * @fn bool SetFilters(const std::uint8_t, const CanardFilter* const)
 * @brief set the can acceptance filters.
 *
 * @param num			number of filter elements
 * @param filters		array of filter settings
 * @return				true on success, false if num is to large
 */
bool hardware::can::SetFilters(const std::uint8_t num, const CanardFilter* const filters)
{
	CANFilter stm_filters[NUM_ACCEPTANCE_FILTERS * HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

	if(num > NUM_ACCEPTANCE_FILTERS) return false;

	for(std::uint_fast8_t i = 0; i < num; i++)
	{
		stm_filters[i].filter = i;
		stm_filters[i].mode = 0;
		stm_filters[i].scale = 1;
		stm_filters[i].assignment = 0;
		stm_filters[i].register1 = filters[i].extended_can_id;
		stm_filters[i].register2 = filters[i].extended_mask;

		if(HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES > 1)
		{
			stm_filters[i + num].filter = i;
			stm_filters[i + num].mode = 0;
			stm_filters[i + num].scale = 1;
			stm_filters[i + num].assignment = 0;
			stm_filters[i + num].register1 = filters[i].extended_can_id;
			stm_filters[i + num].register2 = filters[i].extended_mask;
		}
	}

	canSTM32SetFilters(pcan[0], num, num * HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES, stm_filters);

	return true;
}

/**
 * @fn std::uint32_t GetBitrate(void)
 * @brief get the currently set bitrate
 *
 * @return current bitrate in bits/second
 */
std::uint32_t hardware::can::GetBitrate(void)
{
	return cur_bitrate;
}


