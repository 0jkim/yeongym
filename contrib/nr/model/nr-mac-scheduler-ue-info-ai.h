// Copyright (c) 2024 Seoul National University (SNU)
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "nr-mac-scheduler-ue-info-qos.h"

#include <functional>
#include <unordered_map>
#include <vector>

namespace ns3
{
/**
 * @ingroup scheduler
 * @brief UE representation for a AI-based scheduler
 *
 * The representation stores the weights of a UE, which are also referred to as actions in the RL
 * model, in response to sending the predefined observation. The observation is a vector of
 * LcObservation, each representing an observation of a flow. In addition to RL-related operations,
 * it updates the metrics in NrMacSchedulerUeInfoQos by inheriting from the NrMacSchedulerUeInfoQos
 * class. In resource allocation per symbol, we can design the reward function of a UE using the QoS
 * metrics.
 *
 * @see LcObservation
 * @see Weights
 * @see NrMacSchedulerUeInfoQos
 */
class NrMacSchedulerUeInfoAi : public NrMacSchedulerUeInfoQos
{
  public:
    /**
     * @brief NrMacSchedulerUeInfoAi constructor
     * @param alpha The fairness metric
     * @param rnti RNTI of the UE
     * @param beamId BeamId of the UE
     * @param fn A function that tells how many RB per RBG
     */
    NrMacSchedulerUeInfoAi(float alpha, uint16_t rnti, BeamId beamId, const GetRbPerRbgFn& fn);
    /**
     * @typedef Weights
     * @brief A hash map for weights
     *
     * A hash map for weights that maps a pair of uint8_t to a double.
     * The key is the LC ID, and the value is the weight of the LC as a double.
     */
    typedef std::unordered_map<uint8_t, double> Weights;
    /**
     * @typedef UeWeightsMap
     * @brief A hash map for UE weights
     *
     * A hash map for UE weights that maps a uint8_t to a Weights.
     * The key is the RNTI, and the value is the Weights of the UE.
     */
    typedef std::unordered_map<uint8_t, Weights> UeWeightsMap;

    /**
     * @struct LcObservation
     * @brief A struct for an observation of a flow
     *
     * A struct for an observation of a flow that stores the RNTI, LCG ID, LC ID, QCI, priority, and
     * head-of-line delay of the flow.
     */
    struct LcObservation
    {
        uint16_t rnti;
        uint8_t lcId;
        uint8_t qci;
        uint8_t priority;
        uint16_t holDelay;

        // aoi, cqi state, HarqAckResult 추가
        uint64_t aoi;
        uint8_t cqi;
        // float sinr;
    };

    /**
     * @typedef UpdateAllUeWeightsFn
     * @brief A function type for updating the weights of all UEs.
     */
    using UpdateAllUeWeightsFn = std::function<void(const UeWeightsMap&)>;
    /**
     * @typedef NotifyCb
     * @brief A callback type for notifying with specific parameters.
     *
     * This callback takes the following parameters:
     * - An Observation object representing the observations
     * - A boolean value indicating whether the game is over (true) or not (false)
     * - A float value representing the reward
     * - A string value representing extra information
     * - A pointer to a const NrMacSchedulerOfdmaAi instance
     */
    using NotifyCb = Callback<void,
                              const std::vector<LcObservation>&,
                              bool,
                              float,
                              const std::string&,
                              const UpdateAllUeWeightsFn&>;

    uint64_t GetAoi() const;
    uint8_t GetCqi() const;
    float GetSinrInfo() const;

    void UpdateAoi(uint64_t aoi);
    void UpdateCqi(uint8_t cqi);
    void UpdateHarqAckResult(bool harqAckResult);
    void UpdateSinr(float sinr);

    bool GetHarqAckResult();
    /**
     * @brief Reset DL AI scheduler info
     *
     * Clear the weights for the downlink.
     * It calls also NrMacSchedulerUeInfoQos::ResetDlSchedInfo.
     */
    void ResetDlSchedInfo() override;
    /**
     * @brief Reset UL AI scheduler info
     *
     * Clear the weights for the uplink.
     * It also calls NrMacSchedulerUeInfoQos::ResetUlSchedInfo.
     */
    void ResetUlSchedInfo() override;

    /**
     * @brief Get the current observation for downlink
     * @param ue the UE
     * @return a vector of LcObservation with the current observation
     *
     * Get the current observation for downlink by iterating over the active LCs of the UE.
     * The observation is stored in a vector of LcObservation and each consists of the RNTI,
     * LCG ID, LC ID, QCI, priority, and head-of-line delay of the flow.
     */
    std::vector<LcObservation> GetDlObservation();

    /**
     * @brief Get the current observation for uplink
     * @param ue the UE
     * @return a vector of LcObservation with the current observation
     *
     * Get the current observation for uplink by iterating over the active LCs of the UE.
     * The observation is stored in a vector of LcObservation and each consists of the RNTI,
     * LCG ID, LC ID, QCI, priority, and head-of-line delay of the flow.
     */
    std::vector<LcObservation> GetUlObservation();

    /**
     * @brief Update the weights for downlink
     * @param weights The weights assigned to a UE
     *
     * Update m_weights by copying the weights assigned to a UE.
     * The weights consist of an unordered_map of (key, value) pairs where the lcId is the key,
     * and the weight of the lcId is the value. The higher the weight, the
     * higher the priority of the flow in scheduling.
     */
    void UpdateDlWeights(const Weights& weights);

    /**
     * @brief Update the weights for uplink
     * @param weights the weights assigned to a UE
     *
     * Update m_weights by copying the weights assigned to a UE.
     * The weights consist of an unordered_map of (key, value) pairs where the combination of lcgId
     * and lcId is the key, and the weight of the lcId is the value. The higher the weight, the
     * higher the priority of the flow in scheduling.
     */
    void UpdateUlWeights(const Weights& weights);

    /**
     * @brief Get the reward for downlink
     * @return The reward for the downlink
     *
     * Calculate the reward for the downlink based on the latest observation and the given weights.
     * The reward is calculated as the sum of the rewards of the active LCs.
     * The reward for an LC \( i \) is calculated as
     * \f$ \text{reward}_{i} = \frac{\text{std::pow}(\text{potentialTput}, \alpha)}{\max(1E-9,
     * \text{avgTput})} \times P_{i} \times \text{HOL}_{i} \f$.
     *
     * @alpha is a fairness metric. \( P \) is the priority associated with the QCI.
     * HOL is the head-of-line delay of the LC.
     * Please note that the throughput is calculated in bit/symbol.
     */
    float GetDlReward();

    /**
     * @brief Get the reward for uplink
     * @return The reward for the uplink
     *
     * Calculate the reward for the uplink based on the latest observation and the given weights.
     * The reward is calculated as the sum of the rewards of the active LCs.
     * The reward for an LC \( i \) is calculated as
     * \f$ \text{reward}_{i} = \frac{\text{std::pow}(\text{potentialTput}, \alpha)}{\max(1E-9,
     * \text{avgTput})} \times P_{i} \times \text{HOL}_{i} \f$.
     *
     * @alpha is a fairness metric. \( P \) is the priority associated with the QCI.
     * HOL is the head-of-line delay of the LC.
     * Please note that the throughput is calculated in bit/symbol.
     */
    float GetUlReward();

    /**
     * @brief comparison function object (i.e. an object that satisfies the
     * requirements of Compare) which returns ​true if the first argument is less
     * than (i.e. is ordered before) the second.
     * @param lue Left UE
     * @param rue Right UE
     * @return true if the AI metric of the left UE is higher than the right UE
     *
     * The AI metric is calculated in CalculateDlWeight()
     */
    static bool CompareUeWeightsDl(const NrMacSchedulerNs3::UePtrAndBufferReq& lue,
                                   const NrMacSchedulerNs3::UePtrAndBufferReq& rue);

    /**
     * @brief Calculate the weight of a UE in the downlink
     * @param ue the UE
     * @return the weight of the UE
     *
     * Calculate the weight of a UE in the downlink by iterating over the active LCs of the UE.
     * The weight is calculated as the sum of the weights of the active LCs.
     * The weight of an LC is retrieved from the m_weightsDl map.
     */
    static double CalculateDlWeight(const NrMacSchedulerNs3::UePtrAndBufferReq& ue);

    /**
     * @brief comparison function object (i.e. an object that satisfies the
     * requirements of Compare) which returns ​true if the first argument is less
     * than (i.e. is ordered before) the second.
     * @param lue Left UE
     * @param rue Right UE
     * @return true if the AI metric of the left UE is higher than the right UE
     *
     * The AI metric is calculated in CalculateUlWeight()
     */
    static bool CompareUeWeightsUl(const NrMacSchedulerNs3::UePtrAndBufferReq& lue,
                                   const NrMacSchedulerNs3::UePtrAndBufferReq& rue);

    /**
     * @brief Calculate the weight of a UE in the uplink
     * @param ue the UE
     * @return the weight of the UE
     *
     * Calculate the weight of a UE in the uplink by iterating over the active LCs of the UE.
     * The weight is calculated as the sum of the weights of the active LCs.
     * The weight of an LC is retrieved from the m_weightsUl map.
     */
    static double CalculateUlWeight(const NrMacSchedulerNs3::UePtrAndBufferReq& ue);

    Weights m_weightsDl; //!< Weights assigned to each flow for a UE in the downlink
    Weights m_weightsUl; //!< Weights assigned to each flow for a UE in the uplink

    uint64_t m_aoi = 0;
    uint8_t m_cqi = 0;
    float sinr = 0.0;
    bool m_harqAckResult; // reward 계산 할 때, agent의 action에 대한 outcome shaping을
                          // 위해서 HARQ ACk/NACK 메시지를 확인
};

} // namespace ns3
