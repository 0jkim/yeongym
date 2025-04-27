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

void
NrMacSchedulerUeInfoAi::UpdateHarqAckResult(bool harqAckResult)
{
    m_harqAckResult = harqAckResult;
}

// void
// NrMacSchedulerUeInfoAi::UpdateSinr(float sinr)
// {
//     sinrInfo = sinr;
// }

bool
NrMacSchedulerUeInfoAi::GetHarqAckResult()
{
    return m_harqAckResult;
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

// float
// NrMacSchedulerUeInfoAi::GetSinrInfo() const
// {
//     return sinrInfo;
// }

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
    // float avgSinr = 0.0f;
    // if (!m_ulCqi.m_sinr.empty())
    // {
    //     double sumSinr = 0.0;
    //     for (const auto& sinr : m_ulCqi.m_sinr)
    //     {
    //         sumSinr += sinr;
    //     }
    //     avgSinr = static_cast<float>(sumSinr / m_ulCqi.m_sinr.size());
    //     NS_LOG_UNCOND("Sinr front: " << m_ulCqi.m_sinr.front());
    // }

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
    NS_LOG_UNCOND("[ue-info-ai] <GetUlReward>");

    float totalReward = 0.0f;
    int lcCount = 0;

    for (const auto& ueLcg : m_ulLCG)
    {
        for (const auto lcId : ueLcg.second->GetActiveLCIds())
        {
            const auto& lc = ueLcg.second->GetLC(lcId);

            // 상태값 가져오기
            float normAoi = std::min(m_aoi / 10000.0f, 1.0f); // AoI normalization
            float normCqi = m_ulCqi.m_wbCqi / 15.0f;          // CQI normalization
            bool harqAckResult = false;

            auto it = m_harqAckResult;

            // 기본 reward 구성
            float reward = 0.0f;
            reward += -normAoi;       // AoI 높으면 패널티
            reward += 0.5f * normCqi; // CQI 좋으면 보너스

            NS_LOG_UNCOND("[혹시나 해서 확인해봄] : " << it);
            if (!it)
            {
                reward -= 2.0f; // NACK 받으면 추가 패널티
            }
            else
            {
                reward += 1.0f; // ACK 받으면 추가 보너스
            }

            totalReward += reward;
            lcCount++;
        }
    }

    // 평균 reward로 정규화
    if (lcCount > 0)
    {
        totalReward /= static_cast<float>(lcCount);
    }

    return totalReward;
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

// Agent가 할당한 UE의 weight를 비교
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

    // NS_LOG_UNCOND("lw : " << lw << " rw : " << rw);
    return lw > rw;
}

} // namespace ns3
