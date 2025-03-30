/**
 * Target System Model (TR-38.913)
 * urban coverage for massive connection

             Attributes and values
 * 1. Carrier Frequency : 700 or 2100MHz
 * 2. Inter-Site Distance(ISD) : 500 or 1732m
 * 3. gNB Type : Only Macro
 * 4. Device Deployment : Indoor and Outdoor in - car
   20% users => Outdoor car (under 100km/h) or Outdoor man (under 3km/h)
   80% users => Indoor man (under 3km/h)
 * 5. Service Profile : Non-full buffer with small packets
 */

#include "ns3/antenna-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/grid-scenario-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-rrc.h"

#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("yeongym");

std::fstream m_ScenarioFile;

class MyModel : public Application
{
  public:
    MyModel()
        : m_packetSent(0),
          m_running(true),
          m_sizeRng(CreateObject<UniformRandomVariable>()),
          m_periodRng(CreateObject<UniformRandomVariable>())
    {
        m_sizeRng->SetAttribute("Min", DoubleValue(50));
        m_sizeRng->SetAttribute("Max", DoubleValue(500));
        m_periodRng->SetAttribute("Min", DoubleValue(500));
        m_periodRng->SetAttribute("Max", DoubleValue(5000));
    }

    virtual ~MyModel()
    {
    }

    void Setup(Ptr<NetDevice> device,
               Address address,
               uint32_t nPackets,
               double simTime,
               bool isMobile,
               uint32_t seed)
    {
        m_device = device;
        m_address = address;
        m_nPackets = nPackets;
        m_simTime = simTime;
        m_isMobile = isMobile;
        if (m_isMobile)
        {
            m_sizeRng->SetAttribute("Min", DoubleValue(200));
            m_sizeRng->SetAttribute("Max", DoubleValue(500));
        }
        m_rng.seed(seed);
        ScheduleNextPacket();
    }

    void SendPacketUl()
    {
        uint32_t packetSize = static_cast<uint32_t>(m_sizeRng->GetValue()); // 수정
        Ptr<Packet> pkt = Create<Packet>(packetSize);
        Ipv4Header ipv4Header;
        ipv4Header.SetProtocol(Ipv4L3Protocol::PROT_NUMBER);
        pkt->AddHeader(ipv4Header);

        m_device->Send(pkt, m_address, Ipv4L3Protocol::PROT_NUMBER);
        NS_LOG_INFO("UE send packet of size " << packetSize << "bytes");

        if (++m_packetSent < m_nPackets && m_running)
        {
            ScheduleNextPacket();
        }
    }

    void ScheduleNextPacket()
    {
        Time tNext = MilliSeconds(m_periodRng->GetValue());
        NS_LOG_INFO("m_periodRng : "<<m_periodRng->GetValue());
        Simulator::Schedule(tNext, &MyModel::SendPacketUl, this);
    }

  private:
    Ptr<NetDevice> m_device;
    Address m_address;
    uint32_t m_nPackets, m_packetSent;
    double m_simTime;
    bool m_isMobile;
    bool m_running;
    std::mt19937 m_rng;
    Ptr<UniformRandomVariable> m_sizeRng;   // 패킷 크기 난수 
    Ptr<UniformRandomVariable> m_periodRng; // 패킷 주기 난수 
};

// EnableTracesIfConnected 함수를 main 외부로 이동
void
EnableTracesIfConnected(Ptr<NrHelper> nrHelper, NetDeviceContainer ueNetDev)
{
    bool allConnected = true;
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        if (!ueDev || ueDev->GetRrc()->GetState() != NrUeRrc::CONNECTED_NORMALLY)
        {
            allConnected = false;
            break;
        }
    }
    if (allConnected)
    {
        nrHelper->EnableTraces();
        NS_LOG_INFO("All UEs connected, traces enabled.");
    }
    else
    {
        Simulator::Schedule(MilliSeconds(50), &EnableTracesIfConnected, nrHelper, ueNetDev);
    }
}

// RRC 상태 모니터링 함수 정의
void
MonitorUeConnection(Ptr<NrHelper> nrHelper,
                    NetDeviceContainer ueNetDev,
                    Time initialSetupTime,
                    std::vector<Ptr<MyModel>>& v_modelUl)
{
    bool allConnected = true;
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        if (!ueDev || ueDev->GetRrc()->GetState() != NrUeRrc::CONNECTED_NORMALLY)
        {
            allConnected = false;
            break;
        }
    }
    if (allConnected)
    {
        NS_LOG_INFO("All UEs connected, starting simulation.");
        nrHelper->EnableTraces(); // 트레이싱 활성화
        // 패킷 전송 시작
        for (uint32_t i = 0; i < v_modelUl.size(); i++)
        {
            Simulator::ScheduleNow(&MyModel::SendPacketUl, v_modelUl[i]);
        }
    }
    else
    {
        // 50ms 후 다시 상태 확인
        Simulator::Schedule(MilliSeconds(50),
                            &MonitorUeConnection,
                            nrHelper,
                            ueNetDev,
                            initialSetupTime,
                            v_modelUl);
    }
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("yeongym", LOG_LEVEL_INFO);
    // LogComponentEnable("NrGnbMac", LOG_LEVEL_INFO);
    // LogComponentEnable("NrMacSchedulerCQIManagement", LOG_LEVEL_INFO);

    RngSeedManager::SetSeed(2);  // 시드 값 설정
    RngSeedManager::SetRun(2);   // 런 번호 설정

    double centralFrequencyBand1 = 700e6; // 700MHz 중심 주파수
    double bandwidthBand1 = 10e6;         // 5MHz 주파수 대역 (699.5 - 700.5MHz)
    uint16_t numerologyBwp1 = 0;          // mMTC numerology 0
    uint16_t gNBNum = 1;                  // 1 gNB
    uint16_t ueNum = 3;                  // 100 UE
    double isd = 300.0;                  // ISD 1732m

    bool enableUl = true;     // Uplink on
    uint32_t nPackets = 1000; // Packets Numbers (for simulation)
    double simTime = 60.0;    // Simulation Time

    CommandLine cmd;
    cmd.AddValue("centralFrequencyBand1",
                 "The system frequency to be used in band1",
                 centralFrequencyBand1);
    cmd.AddValue("bandwidthBand1", "The system bandwidth to be used in band1", bandwidthBand1);
    cmd.AddValue("numerologyBwp1", "The numerology to be used in bandwidth part 1", numerologyBwp1);
    cmd.AddValue("gNBNum", "The system gNB number", gNBNum);
    cmd.AddValue("ueNum", "The system ue number", ueNum);
    cmd.AddValue("isd", "The Distance from gNB to ue", isd);
    cmd.AddValue("enableUl", "Enable Uplink", enableUl);
    cmd.AddValue("nPackets", "Number of paackets in each ue", nPackets);
    cmd.AddValue("simTime", "The simulation time", simTime);
    cmd.Parse(argc, argv);

    /**
     * Configuration UE Mobility
     * Deployment UE, gNB
     * Indoor 80%, Outdoor 20%
     */
    // Mobility Helper
    NodeContainer gnbNodes;
    gnbNodes.Create(gNBNum);
    NodeContainer ueNodes;
    ueNodes.Create(ueNum * gNBNum);

    // gNB mobility
    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                     "MinX",
                                     DoubleValue(300.0),
                                     "MinY",
                                     DoubleValue(-300.0),
                                     "GridWidth",
                                     UintegerValue(1),
                                     "LayoutType",
                                     StringValue("RowFirst"));
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);
    for (uint32_t i = 0; i < gNBNum; i++)
    {
        Ptr<MobilityModel> mob = gnbNodes.Get(i)->GetObject<MobilityModel>();
        if (!mob)
        {
            NS_FATAL_ERROR("No MobilityModel installed for gNB " << i);
        }
        mob->SetPosition(Vector(300.0, -300.0, 10.0));
        NS_LOG_INFO("✅ gNB " << i << " mobility model initialized.");
    }

    // UE mobility
    MobilityHelper ueMobility;
    // ueMobility.SetPositionAllocator("ns3::GridPositionAllocator",
    //                                 "MinX",
    //                                 DoubleValue(-2.5),
    //                                 "MinY",
    //                                 DoubleValue(-2.5),
    //                                 "GridWidth",
    //                                 UintegerValue(10),
    //                                 "LayoutType",
    //                                 StringValue("RowFirst"));
    /**
     * UE 배치 test (for tbler scenario)
     */
    ueMobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                    "X", DoubleValue(300.0),
                                    "Y", DoubleValue(-300.0),
                                    "rho", DoubleValue(100.0));
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);

    // UE speed setting
    std::vector<bool> isMobile(ueNum * gNBNum, false);
    std::vector<uint32_t> indices(ueNum * gNBNum);
    for (uint32_t i = 0; i < ueNum * gNBNum; i++)
    {
        indices[i] = i;
    }
    std::shuffle(indices.begin(), indices.end(), std::mt19937(std::random_device()()));
    for (uint32_t i = 0; i < ueNum * gNBNum; i++)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            ueNodes.Get(indices[i])->GetObject<ConstantVelocityMobilityModel>();
        if (!mob)
        {
            NS_FATAL_ERROR("No MobilityModel installed for UE " << i);
        }
        if (i < (ueNum * gNBNum * 0.2)) // 20% 차량 UE (60km/h)
        {
            mob->SetVelocity(Vector(16.6, 0, 0));
            isMobile[indices[i]] = true;
            NS_LOG_INFO("UE " << indices[i] << " set to 60km/h (Outdoor)");
        }
        else if (i < (ueNum * gNBNum * 0.6)) // 40% 고정 UE (3km/h)
        {
            mob->SetVelocity(Vector(0.833, 0, 0)); // 3km/h
            NS_LOG_INFO("UE " << indices[i] << " set to 3km/h (Indoor)");
        }
        else
        {
            mob->SetVelocity(Vector(0, 0, 0));
            NS_LOG_INFO("UE " << indices[i] << " set to 0km/h (Indoor)");
        }
    }

    // Configuration NR (P2P or IdleBeamforming => here. P2P)
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // Set Scheduler Type
    /**
     * TODO
     * 1. AoI Greedy
     * 2. Reinforcement Learning
     */
    /**
     * select_sch is scheduling algorithms option
     * 1 : OFDMA Round Robin
     * 2 : OFDMA Proportional Fair
     * 3 : Greedy AoI
     * 4 : RL based
     */
    int select_sch = 3;
    if (select_sch == 1)
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    }
    else if (select_sch == 2)
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaPF::GetTypeId());
    }
    else if (select_sch == 3)
    {
        /**
         * AoI greedy Scheduler
         */
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaGreedy::GetTypeId());
    }
    // else if (select_sch == 4)
    // {
    /**
     * RL based Scheduler
     */
    // }

    // For data drop
    // Disable SRS (for data drop)
    nrHelper->SetSchedulerAttribute("SrsSymbols", UintegerValue(1));

    // HARQ setting
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    // Config::SetDefault("ns3::NrHelper::HarqEnabled", BooleanValue(false));

    // Set channel update period
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100))); // 100ms 마다 채널 상태 갱신

    // Set mcs
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(8));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(2));

    // Set for pathloss and shadowing (에러 없는 환경은 false)
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    
    // Set error model
    std::string errorModel = "ns3::NrEesmIrT1";
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    // Set fading
    bool fadingEnabled = true;
    auto bandMask = NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL;
    if (fadingEnabled)
    {
        bandMask |= NrHelper::INIT_FADING;
    }

    // Bandwidth environment for urban coverage for massive connection
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand,
                                                    BandwidthPartInfo::UMa_nLoS);
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    nrHelper->InitializeOperationBand(&band1, bandMask);
    allBwps = CcBwpCreator::GetAllBwps({band1});

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<ThreeGppAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    // Install NetDevice
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Set the attribute of the netdevice (enbNetDev.Get (0)) and bandwidth part (0)
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // RandomStream setting
    int64_t randomStream = 1;
    randomStream += ueMobility.AssignStreams(ueNodes, randomStream);
    randomStream += gnbMobility.AssignStreams(gnbNodes, randomStream);
    randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Set IP
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // Attach UE and gNB
    nrHelper->AttachToClosestGnb(ueNetDev, enbNetDev);

    // RRC Debugger
    Simulator::Schedule(MilliSeconds(150), &EnableTracesIfConnected, nrHelper, ueNetDev);

    // Initial Delay
    Time initialSetupTime = Seconds(3);

    // Set Uplink Traffic
    std::vector<Ptr<MyModel>> v_modelUl(ueNum);
    std::random_device rd;
    for (uint32_t i = 0; i < ueNum; i++)
    {
        v_modelUl[i] = CreateObject<MyModel>();
        v_modelUl[i]->Setup(ueNetDev.Get(i),
                            enbNetDev.Get(0)->GetAddress(),
                            nPackets,
                            simTime,
                            isMobile[i],
                            rd());
        Simulator::Schedule(initialSetupTime + MicroSeconds(100 * i),
                            &MyModel::SendPacketUl,
                            v_modelUl[i]);
    }

    // Simulator::Schedule(initialSetupTime, &NrHelper::EnableTraces, nrHelper);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}