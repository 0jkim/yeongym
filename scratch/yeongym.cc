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
#include "ns3/nr-mac-scheduler-ai-ns3-gym-env.h"
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
        NS_LOG_INFO("m_periodRng : " << m_periodRng->GetValue());
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

// 위치 변경 시 호출되는 콜백 함수 정의
void
UePositionLogger(std::string context, Ptr<const MobilityModel> mobility)
{
    Vector pos = mobility->GetPosition();
    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s - " << context << " Position: (" << pos.x
                                                << ", " << pos.y << ", " << pos.z << ")");
}

int
main(int argc, char* argv[])
{
    // LogComponentEnable("yeongym", LOG_LEVEL_INFO);
    // LogComponentEnable("NrGnbMac", LOG_LEVEL_INFO);
    // LogComponentEnable("NrMacSchedulerCQIManagement", LOG_LEVEL_INFO);
    // LogComponentEnable("NrSpectrumPhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrUeRrc", LOG_LEVEL_INFO);
    LogComponentEnable("NrGnbRrc", LOG_LEVEL_INFO);
    RngSeedManager::SetSeed(2); // 시드 값 설정
    RngSeedManager::SetRun(2);  // 런 번호 설정

    double centralFrequencyBand1 = 700e6; // 700MHz 중심 주파수
    double bandwidthBand1 = 10e6;         // 5MHz 주파수 대역 (699.5 - 700.5MHz)
    uint16_t numerologyBwp1 = 0;          // mMTC numerology 0
    uint16_t gNBNum = 1;                  // 1 gNB
    uint16_t ueNum = 30;                  // 100 UE
    double isd = 300.0;                   // ISD 1732m

    bool enableUl = true;     // Uplink on
    uint32_t nPackets = 1000; // Packets Numbers (for simulation)
    double simTime = 320;     // Simulation Time

    uint32_t openGymPort = 5555;

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
    cmd.AddValue("openGymPort", "OpenGym communication port", openGymPort);

    cmd.Parse(argc, argv);

    /**
     * Configuration UE Mobility
     * Deployment UE, gNB
     * Indoor 80%, Outdoor 20%
     * 40% UE = 0km/h
     * 40% UE = 3km/h
     * 20% UE = 60km/h
     */
    // === Mobility Helper ===
    NodeContainer gnbNodes;
    gnbNodes.Create(gNBNum);
    NodeContainer ueNodes;
    ueNodes.Create(ueNum * gNBNum);

    // === gNB mobility ===
    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                     "MinX",
                                     DoubleValue(500.0),
                                     "MinY",
                                     DoubleValue(-500.0),
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
        mob->SetPosition(Vector(500.0, -500.0, 10.0));
        NS_LOG_INFO("✅ gNB " << i << " mobility model initialized.");
    }

    // === UE mobility ===
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    ueMobility.Install(ueNodes);

    // === 초기 위치 설정 (원 내부 랜덤) ===
    double centerX = 500.0, centerY = -500.0, radius = 100.0;
    Ptr<UniformDiscPositionAllocator> discAlloc = CreateObject<UniformDiscPositionAllocator>();
    discAlloc->SetX(centerX);
    discAlloc->SetY(centerY);
    discAlloc->SetRho(radius);

    std::vector<Vector> initialPositions;
    for (uint32_t i = 0; i < ueNum * gNBNum; i++)
    {
        Vector pos = discAlloc->GetNext();
        initialPositions.push_back(pos);
        Ptr<WaypointMobilityModel> mob = ueNodes.Get(i)->GetObject<WaypointMobilityModel>();
        mob->AddWaypoint(Waypoint(Seconds(0), pos));
    }

    // === 이동성 설정 (유지 상황에 따른 이동 속도) ===
    std::vector<bool> isMobile(ueNum * gNBNum, false);

    for (uint32_t i = 0; i < ueNum * gNBNum; i++)
    {
        Ptr<WaypointMobilityModel> mob = ueNodes.Get(i)->GetObject<WaypointMobilityModel>();
        Vector current = initialPositions[i];

        double speed = 0.0;
        if (i < ueNum * gNBNum * 0.2)
        {
            speed = 16.6; // 60 km/h
            isMobile[i] = true;
        }
        else if (i < ueNum * gNBNum * 0.6)
        {
            speed = 0.833; // 3 km/h
        }
        else
        {
            speed = 0.0; // 정지
        }

        double now = 0.5;
        Ptr<UniformRandomVariable> pauseRng = CreateObject<UniformRandomVariable>();
        pauseRng->SetAttribute("Min", DoubleValue(0.5));
        pauseRng->SetAttribute("Max", DoubleValue(3.0));

        while (now < simTime && speed > 0)
        {
            Vector dest = discAlloc->GetNext();
            double dist = CalculateDistance(current, dest);
            double travel = dist / speed;

            now += travel;
            if (now > simTime)
                break;

            mob->AddWaypoint(Waypoint(Seconds(now), dest));
            current = dest;

            now += pauseRng->GetValue();
        }
    }

    // Configuration NR (P2P or IdleBeamforming => here. P2P)
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // Set Scheduler Type
    /**
     * TODO
     * 1. AoI Greedy (Fin.)
     * 2. Reinforcement Learning (해야함)
     *
     * select_sch is scheduling algorithms option
     * 1 : OFDMA Round Robin
     * 2 : OFDMA Proportional Fair
     * 3 : OFDMA AoiGreedy
     * 4 : OFDMA Ai (Reinforcement Learning)
     */
    int select_sch = 4;
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
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaGreedy::GetTypeId());
    }
    else if (select_sch == 4)
    {
        std::cout << "[yeongym] <Scheduler Type : OfdmaAi>\n";
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaAi::GetTypeId());

        // AI 환경 객체 생성
        Ptr<NrMacSchedulerAiNs3GymEnv> env = CreateObject<NrMacSchedulerAiNs3GymEnv>();

        Ptr<OpenGymInterface> interface = CreateObject<OpenGymInterface>(openGymPort);
        env->SetOpenGymInterface(interface);

        nrHelper->SetSchedulerAttribute(
            "NotifyCbUl",
            CallbackValue(MakeCallback(&NrMacSchedulerAiNs3GymEnv::NotifyCurrentIteration, env)));
        nrHelper->SetSchedulerAttribute("ActiveUlAi", BooleanValue(true));
    }

    // For data drop
    // === SrsSymbol setting ===
    nrHelper->SetSchedulerAttribute("SrsSymbols", UintegerValue(4));

    // === HARQ retransmit packets setting ===
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    // Config::SetDefault("ns3::NrHelper::HarqEnabled", BooleanValue(false));

    // === Channel update period setting ===
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100))); // 100ms 마다 채널 상태 갱신

    // === Starting mcs setting ===
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(4));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(2));

    // === pathloss and shadowing setting ===
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    // === Error model setting ===
    std::string errorModel = "ns3::NrEesmIrT1";
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    // === Fading setting ===
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
                                                    BandwidthPartInfo::UMa_LoS);
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    nrHelper->InitializeOperationBand(&band1, bandMask);
    allBwps = CcBwpCreator::GetAllBwps({band1});

    // Setting tx power
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));  // UE: 23 dBm
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(43.0)); // gNB: 43 dBm

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

    // IP setting
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // 이동성 Callback Function
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        std::ostringstream oss;
        oss << "/NodeList/" << ue->GetId() << "/$ns3::MobilityModel/CourseChange";
        Config::Connect(oss.str(), MakeCallback(&UePositionLogger));
    }

    // RRC Debugger
    Simulator::Schedule(MilliSeconds(150), &EnableTracesIfConnected, nrHelper, ueNetDev);

    // Initial Delay
    Time initialSetupTime = Seconds(3);

    // Init delay random value
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Min", DoubleValue(50000.0));  // 단위: 마이크로초 (us) → 50ms
    rng->SetAttribute("Max", DoubleValue(200000.0)); // 단위: 마이크로초 (us) → 200ms

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

        Time randomStart = initialSetupTime + MicroSeconds(rng->GetValue());
        Simulator::Schedule(randomStart, &MyModel::SendPacketUl, v_modelUl[i]);
    }

    // Attach UE and gNB
    nrHelper->AttachToClosestGnb(ueNetDev, enbNetDev);

    // Simulator::Schedule(initialSetupTime, &NrHelper::EnableTraces, nrHelper);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    std::cout << "\n FIN. " << std::endl;

    Simulator::Destroy();

    return 0;
}