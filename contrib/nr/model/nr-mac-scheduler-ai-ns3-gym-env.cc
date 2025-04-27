// Copyright (c) 2024 Seoul National University (SNU)
// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ai-ns3-gym-env.h"

#include "ns3/log.h"

#ifdef HAVE_OPENGYM

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrMacSchedulerAiNs3GymEnv");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerAiNs3GymEnv);

NrMacSchedulerAiNs3GymEnv::NrMacSchedulerAiNs3GymEnv()
{
    NS_LOG_FUNCTION(this);
    m_numFlows = 0;
    m_gameOver = false;
    m_reward = 0.0;
}

NrMacSchedulerAiNs3GymEnv::NrMacSchedulerAiNs3GymEnv(uint32_t numFlows)
{
    NS_LOG_FUNCTION(this);
    m_numFlows = numFlows;
    m_gameOver = false;
    m_reward = 0.0;
}

NrMacSchedulerAiNs3GymEnv::~NrMacSchedulerAiNs3GymEnv()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrMacSchedulerAiNs3GymEnv::GetTypeId()
{
    static TypeId tid = TypeId("NrMacSchedulerAiNs3GymEnv")
                            .SetParent<OpenGymEnv>()
                            .AddConstructor<NrMacSchedulerAiNs3GymEnv>();
    return tid;
}

void
NrMacSchedulerAiNs3GymEnv::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
NrMacSchedulerAiNs3GymEnv::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    float low = 0.0;
    float high = 1.0; // agent가 weight를 0-1사이의 실수 값으로 주기 위해 최댓값을 1.0으로 설정정
    std::vector<uint32_t> shape = {m_numFlows};
    std::string dtype = TypeNameGet<float>();
    return Create<OpenGymBoxSpace>(low, high, shape, dtype);
}

Ptr<OpenGymSpace>
NrMacSchedulerAiNs3GymEnv::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    float low = 0.0;
    float high = 100.0;
    std::vector<uint32_t> shape = {
        m_numFlows,
        6,
    };
    std::string dtype = TypeNameGet<uint16_t>();
    return Create<OpenGymBoxSpace>(low, high, shape, dtype);
}

bool
NrMacSchedulerAiNs3GymEnv::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return m_gameOver;
}

Ptr<OpenGymDataContainer>
NrMacSchedulerAiNs3GymEnv::GetObservation()
{
    NS_LOG_FUNCTION(this);
    std::vector<uint32_t> shape = {
        m_numFlows,
        6,
    };
    Ptr<OpenGymBoxContainer<uint16_t>> observation =
        CreateObject<OpenGymBoxContainer<uint16_t>>(shape);
    for (auto& obs : m_observation)
    {
        observation->AddValue(obs.rnti);
        observation->AddValue(obs.lcId);
        observation->AddValue(obs.priority);
        observation->AddValue(obs.holDelay);
        observation->AddValue(obs.aoi);
        observation->AddValue(obs.cqi);
        // observation->AddValue(obs.sinr);
    }
    return observation;
}

float
NrMacSchedulerAiNs3GymEnv::GetReward()
{
    NS_LOG_FUNCTION(this);
    return m_reward;
}

std::string
NrMacSchedulerAiNs3GymEnv::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);

    return m_extraInfo;
}

/**
 * 에이전트가 전달한 action을 수행하는 메서드
 */
bool
NrMacSchedulerAiNs3GymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymBoxContainer<float>> actionBox = DynamicCast<OpenGymBoxContainer<float>>(action);
    std::vector<float> actionData = actionBox->GetData();
    NrMacSchedulerUeInfoAi::UeWeightsMap ueWeightsMap;
    for (uint32_t i = 0; i < m_numFlows; i++)
    {
        if (ueWeightsMap.end() == ueWeightsMap.find(m_observation[i].rnti))
        {
            ueWeightsMap[m_observation[i].rnti] = NrMacSchedulerUeInfoAi::Weights();
            NS_LOG_UNCOND("[gym-env] <ExecuteActions> : Rnti "
                          << m_observation[i].rnti << " received weight " << actionData[i]
                          << " from agent");
        }
        ueWeightsMap[m_observation[i].rnti][m_observation[i].lcId] = actionData[i];
    }
    m_updateAllUeWeightsFn(ueWeightsMap);
    return true;
}

void
NrMacSchedulerAiNs3GymEnv::NotifyCurrentIteration(
    const std::vector<NrMacSchedulerUeInfoAi::LcObservation>& observations,
    bool isGameOver,
    float reward,
    const std::string& extraInfo,
    const NrMacSchedulerUeInfoAi::UpdateAllUeWeightsFn& updateAllUeWeightsFn)
{
    NS_LOG_FUNCTION(this);
    if (m_numFlows != observations.size())
    {
        m_numFlows = observations.size();
    }
    m_observation = observations;
    m_gameOver = isGameOver;
    m_reward = reward;
    m_extraInfo = extraInfo;
    m_updateAllUeWeightsFn = updateAllUeWeightsFn;
    m_currentStep++;
    // if (m_currentStep >= m_maxSteps)
    // {
    //     m_gameOver = true;
    // }
    Notify();
}

} // namespace ns3

#endif // HAVE_OPENGYM
