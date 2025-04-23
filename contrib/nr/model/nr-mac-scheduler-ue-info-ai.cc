#include "nr-mac-scheduler-ue-info-ai.h"

#include "ns3/log.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerUeInfoAi");

NrMacSchedulerUeInfoAi::NrMacSchedulerUeInfoAi(float alpha,
                                               uint16_t rnti,
                                               BeamId beamId,
                                               const GetRbPerRbgFn& fn)
    : NrMacSchedulerUeInfoQos(alpha, rnti, beamId, fn)
{
}

void
NrMacSchedulerUeInfoAi::ResetDlSchedInfo()
{
    m_weightsDl.clear();
    NrMacSchedulerUeInfoQos::ResetDlSchedInfo();
}

void
NrMacSchedulerUeInfoAi::ResetUlSchedInfo()
{
    m_weightsUl.clear();
    NrMacSchedulerUeInfoQos::ResetUlSchedInfo();
}

void
NrMacSchedulerUeInfoAi::UpdateDlWeights(const Weights& weights)
{
    m_weightsDl = weights;
}

void
NrMacSchedulerUeInfoAi::UpdateUlWeights(const Weights& weights)
{
    m_weightsUl = weights;
}

void
NrMacSchedulerUeInfoAi::UpdateAoi(uint64_t aoi)
{
    m_aoi = aoi;
}

void
NrMacSchedulerUeInfoAi::UpdateCqi(uint8_t cqi)
{
    m_cqi = cqi;
}

uint64_t
NrMacSchedulerUeInfoAi::GetAoi() const
{
    return m_aoi;
}

uint8_t
NrMacSchedulerUeInfoAi::GetCqi() const
{
    return m_cqi;
}

std::vector<NrMacSchedulerUeInfoAi::LcObservation>
NrMacSchedulerUeInfoAi::GetDlObservation()
{
    std::vector<LcObservation> observations;
    for (const auto& ueLcg : m_dlLCG)
    {
        for (const auto lcId : ueLcg.second->GetActiveLCIds())
        {
            const auto& lc = ueLcg.second->GetLC(lcId);
            observations.push_back({m_rnti,
                                    lcId,
                                    lc->m_qci,
                                    lc->m_priority,
                                    lc->m_rlcTransmissionQueueHolDelay,
                                    m_aoi,
                                    m_dlCqi.m_wbCqi});
        }
    }
    return observations;
}

std::vector<NrMacSchedulerUeInfoAi::LcObservation>
NrMacSchedulerUeInfoAi::GetUlObservation()
{
    std::vector<LcObservation> observations;
    for (const auto& ueLcg : m_ulLCG)
    {
        for (const auto lcId : ueLcg.second->GetActiveLCIds())
        {
            const auto& lc = ueLcg.second->GetLC(lcId);
            observations.push_back({m_rnti,
                                    lcId,
                                    lc->m_qci,
                                    lc->m_priority,
                                    lc->m_rlcTransmissionQueueHolDelay,
                                    m_aoi,
                                    m_ulCqi.m_wbCqi});
        }
    }
    return observations;
}

float
NrMacSchedulerUeInfoAi::GetDlReward()
{
    float reward = 0.0;
    for (const auto& ueLcg : m_dlLCG)
    {
        for (const auto lcId : ueLcg.second->GetActiveLCIds())
        {
            const auto& lc = ueLcg.second->GetLC(lcId);
            if (m_avgTputDl == 0 || lc->m_rlcTransmissionQueueHolDelay == 0)
                continue;

            reward +=
                std::pow(m_potentialTputDl, m_alpha) /
                (std::max(1E-9, m_avgTputDl) * lc->m_priority * lc->m_rlcTransmissionQueueHolDelay);
        }
    }
    return reward;
}

float
NrMacSchedulerUeInfoAi::GetUlReward()
{
    float reward = 0.0;
    for (const auto& ueLcg : m_ulLCG)
    {
        for (const auto lcId : ueLcg.second->GetActiveLCIds())
        {
            const auto& lc = ueLcg.second->GetLC(lcId);
            if (m_avgTputUl == 0 || lc->m_rlcTransmissionQueueHolDelay == 0)
                continue;

            reward +=
                std::pow(m_potentialTputUl, m_alpha) /
                (std::max(1E-9, m_avgTputUl) * lc->m_priority * lc->m_rlcTransmissionQueueHolDelay);
        }
    }
    return reward;
}

bool
NrMacSchedulerUeInfoAi::CompareUeWeightsDl(const NrMacSchedulerNs3::UePtrAndBufferReq& lue,
                                           const NrMacSchedulerNs3::UePtrAndBufferReq& rue)
{
    auto lPtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(lue.first.get());
    auto rPtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(rue.first.get());

    double lw = 0.0;
    for (const auto& lc : lue.first->m_dlLCG)
        for (const auto& id : lc.second->GetActiveLCIds())
            lw += lPtr->m_weightsDl[id];

    double rw = 0.0;
    for (const auto& lc : rue.first->m_dlLCG)
        for (const auto& id : lc.second->GetActiveLCIds())
            rw += rPtr->m_weightsDl[id];

    return lw > rw;
}

bool
NrMacSchedulerUeInfoAi::CompareUeWeightsUl(const NrMacSchedulerNs3::UePtrAndBufferReq& lue,
                                           const NrMacSchedulerNs3::UePtrAndBufferReq& rue)
{
    auto lPtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(lue.first.get());
    auto rPtr = dynamic_cast<NrMacSchedulerUeInfoAi*>(rue.first.get());

    double lw = 0.0;
    for (const auto& lc : lue.first->m_ulLCG)
        for (const auto& id : lc.second->GetActiveLCIds())
            lw += lPtr->m_weightsUl[id];

    double rw = 0.0;
    for (const auto& lc : rue.first->m_ulLCG)
        for (const auto& id : lc.second->GetActiveLCIds())
            rw += rPtr->m_weightsUl[id];

    return lw > rw;
}

} // namespace ns3
