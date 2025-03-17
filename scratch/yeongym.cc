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

#include "ns3/core-module.h"
#include "ns3/antenna-module.h"
#include "ns3/config-store.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/grid-scenario-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("yeongym");

std::fstream m_ScenarioFile;

class MyModel : public Application
{
    public:
        MyModel() : m_packetSent(0),m_running(true){}

        virtual ~MyModel(){}

        void Setup(Ptr<NetDevice> device,
                Address address,
                uint32_t nPackets,
                double simTime,
                bool isMobile, uint32_t seed)
        {
            m_device = device;
            m_address = address;
            m_nPackets = nPackets;
            m_simTime = simTime;
            m_isMobile = isMobile;
            m_rng.seed(seed);
            ScheduleNextPacket();
        }
        
        void SendPacketUl()
        {
            std::uniform_int_distribution<uint32_t> sizeDist(m_isMobile ? 200 : 50, m_isMobile ? 500 : 200);
            uint32_t packetSize = sizeDist(m_rng);
            Ptr<Packet> pkt = Create<Packet>(packetSize);
            m_device->Send(pkt, m_address, Ipv4L3Protocol::PROT_NUMBER);
            NS_LOG_INFO("UE send packet of size " << packetSize << "bytes");

            if(++m_packetSent < m_nPackets && m_running)
            {
                ScheduleNextPacket();
            }
        }
    
    private:
        void ScheduleNextPacket()
        {
            std::uniform_int_distribution<uint32_t> periodDist(10, 5000);
            Time tNext = MilliSeconds(periodDist(m_rng));
            Simulator::Schedule(tNext, &MyModel::SendPacketUl, this);
        }

        Ptr<NetDevice> m_device;
        Address m_address;
        uint32_t m_nPackets, m_packetSent;
        double m_simTime;
        bool m_isMobile;
        bool m_running;
        std::mt19937 m_rng;
};

int main(int argc, char* argv[])
{

    double centralFrequencyBand1 = 700e6; // 700MHz 중심 주파수
    double bandwidthBand1 = 10e6;          // 5MHz 주파수 대역 (699.5 - 700.5MHz)
    uint16_t numerologyBwp1 = 0;          // mMTC numerology 0
    uint16_t gNBNum = 1;                  // 1 gNB
    uint16_t ueNum = 100;                 // 100 UE
    double isd = 1732.0;                  // ISD 1732m

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
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNBNum);
    gridScenario.SetHorizontalBsDistance(isd);
    gridScenario.SetBsHeight(10.0);
    gridScenario.SetUtHeight(1.5);
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(gNBNum);
    gridScenario.SetUtNumber(ueNum);
    gridScenario.CreateScenario();

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(-isd / 2),
                                  "MinY",
                                  DoubleValue(-isd / 2),
                                  "GridWidth",
                                  UintegerValue(1),
                                  "LayoutType",
                                  StringValue("RowFirst"));

    NodeContainer ueNodes = gridScenario.GetUserTerminals();
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    // 디버깅: 설치 확인
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
        if (!mob)
        {
            NS_FATAL_ERROR("No MobilityModel installed for UE " << i);
        }
        NS_LOG_INFO("UE " << i << " has mobility model: " << mob->GetInstanceTypeId().GetName());
    }

    std::vector<bool> isMobile(ueNum, false);
    std::vector<uint32_t> indices(ueNum);
    for (uint32_t i=0;i<ueNum;i++)
    {
        indices[i]=i;
    }
    std::shuffle(indices.begin(), indices.end(), std::mt19937(std::random_device()()));

    for (uint32_t i = 0; i < ueNum; i++)
    {
        Ptr<ConstantVelocityMobilityModel> mob = ueNodes.Get(indices[i])->GetObject<ConstantVelocityMobilityModel>();
        if (!mob)
        {
            NS_FATAL_ERROR("ConstantVelocityMobilityModel not found for UE " << indices[i]);
        }
        if (i < ueNum * 0.2)
        {
            mob->SetVelocity(Vector(27.28, 0, 0));
            isMobile[indices[i]]=true;
            NS_LOG_INFO("UE " << i << " set to 100km/h (car)");
        }
        else
        {
            mob->SetVelocity(Vector(0, 0, 0));
            NS_LOG_INFO("UE " << indices[i] << " set to 0km/h (Indoor)");
        }
    }
    
    // // Only Mobility Helper start
    // NodeContainer gnbNodes;
    // gnbNodes.Create(gNBNum);
    // NodeContainer ueNodes;
    // ueNodes.Create(ueNum * gNBNum);
    
    // // gNB 모빌리티
    // MobilityHelper gnbMobility;
    // gnbMobility.SetPositionAllocator("ns3::GridPositionAllocator",
    //                                  "MinX", DoubleValue(0),
    //                                  "MinY", DoubleValue(0),
    //                                  "GridWidth", UintegerValue(1),
    //                                  "LayoutType", StringValue("RowFirst"));
    // gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // gnbMobility.Install(gnbNodes);

    // for (uint32_t i = 0; i < gNBNum; i++)
    // {
    //     Ptr<MobilityModel> mob = gnbNodes.Get(i)->GetObject<MobilityModel>();
    //     if (!mob)
    //     {
    //         NS_FATAL_ERROR("No MobilityModel installed for gNB " << i);
    //     }
    //     mob->SetPosition(Vector(i * 5.0, 0, 10.0));
    //     NS_LOG_INFO("✅ gNB " << i << " mobility model initialized.");
    // }

    // // UE 모빌리티
    // MobilityHelper ueMobility;
    // ueMobility.SetPositionAllocator("ns3::GridPositionAllocator",
    //                                 "MinX", DoubleValue(-2.5),
    //                                 "MinY", DoubleValue(-2.5),
    //                                 "GridWidth", UintegerValue(10),
    //                                 "LayoutType", StringValue("RowFirst"));
    // ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    // ueMobility.Install(ueNodes);
    
    // // UE 속도 설정
    // std::vector<bool> isMobile(ueNum * gNBNum, false);
    // std::vector<uint32_t> indices(ueNum * gNBNum);
    // for (uint32_t i = 0; i <ueNum * gNBNum; i++)
    // {
    //     indices[i] = i;
    // }
    // std::shuffle(indices.begin(), indices.end(), std::mt19937(std::random_device()()));
    // for (uint32_t i = 0; i < ueNum * gNBNum; i++)
    // {
    //     Ptr<ConstantVelocityMobilityModel> mob = ueNodes.Get(indices[i])->GetObject<ConstantVelocityMobilityModel>();
    //     if (!mob)
    //     {
    //         NS_FATAL_ERROR("No MobilityModel installed for UE " << i);
    //     }
    //     if (i < (ueNum * gNBNum * 0.2))
    //     {
    //         mob->SetVelocity(Vector(27.78, 0, 0));
    //         isMobile[indices[i]] = true;
    //         NS_LOG_INFO("UE " << indices[i] << " set to 100km/h (Outdoor)");
    //     }
    //     else
    //     {
    //         mob->SetVelocity(Vector(0, 0, 0));
    //         NS_LOG_INFO("UE " << indices[i] << " set to 0km/h (Indoor)");
    //     }
    // }

    // Configuration NR
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
    int select_sch = 1;
    if (select_sch == 1)
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    }
    else if (select_sch == 2)
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaPF::GetTypeId());
    }
    // else if (select_sch == 3)
    // {

    // }
    // else if (select_sch == 4)
    // {

    // }
    
    // Disable SRS (for data drop)
    nrHelper->SetSchedulerAttribute("SrsSymbols", UintegerValue(0));

    // Disable HARQ (for data drop)
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(false));
    // Config::SetDefault("ns3::NrHelper::HarqEnabled", BooleanValue(false));
    
    // Set channel update period
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));    // 100ms 마다 채널 상태 갱신

    // Set mcs
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(12));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(4));
    
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
    
    // 디버깅 시작
    NS_LOG_INFO("Number of BWPs: " << allBwps.size());
    for (const auto& bwpRef : allBwps)
    {
        const BandwidthPartInfo* bwp = bwpRef.get().get(); // std::reference_wrapper에서 포인터 추출
        NS_LOG_INFO("BWP Bandwidth: " << bwp->m_channelBandwidth << " Hz");
    }
    // 디버깅 끝

    // Install NetDevice
    NetDeviceContainer enbNetDev =
        nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(gridScenario.GetUserTerminals(), allBwps);
    
    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
    nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (4));
    nrHelper->SetUeAntennaAttribute ("AntennaElement",
                                    PointerValue (CreateObject<IsotropicAntennaModel> ()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
    nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (4));
    nrHelper->SetGnbAntennaAttribute ("AntennaElement",
                                        PointerValue (CreateObject<IsotropicAntennaModel> ()));
    // NetDevice 설치
    // NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    // NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->GetGnbPhy (enbNetDev.Get (0), 0)->SetAttribute ("Numerology", UintegerValue (0));

    Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(enbNetDev.Get(0));
    if (gnbDev)
    {
        Ptr<NrGnbPhy> phy = gnbDev->GetPhy(0);
        if (phy)
        {
            NS_LOG_INFO("✅ PHY Found!");
            phy->Initialize(); // 이때 ConfigureMac()이 호출되어야 함
        }
        else
        {
            NS_LOG_ERROR("❌ Failed to get PHY from gNB device");
        }
        
        Ptr<NrGnbMac> mac = gnbDev->GetMac(0);
        if (mac)
        {
            NS_LOG_INFO("✅ MAC Found!");
        }
        else
        {
            NS_LOG_ERROR("❌ Failed to get MAC from gNB device");
        }
    }
    else
    {
        NS_LOG_ERROR("Failed to cast NetDevice to NrGnbNetDevice");
    }
    // Set the attribute of the netdevice (enbNetDev.Get (0)) and bandwidth part (0)
    nrHelper->GetGnbPhy (enbNetDev.Get (0), 0)
        ->SetAttribute ("Numerology", UintegerValue (numerologyBwp1));

    for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
        {
        DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
        }

    for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
        {
        DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }
    // Set IP
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);
    
    if (ueNetDev.GetN() > 0)
    {
        NS_LOG_INFO("✅ UE Devices "<<ueNetDev.GetN()<< " successfully created.");
    }
    else
    {
        NS_LOG_ERROR("❌ UE Devices not created!");
    }

    // Attach UE and gNB
    nrHelper->AttachToClosestGnb(ueNetDev, enbNetDev);

    // 초기 지연 테스트 시작
    Time initialSetupTime = MilliSeconds(100);
    // 테스트 끝

    // Set Uplink Traffic
    std::vector<Ptr<MyModel>> v_modelUl(ueNum);
    std::random_device rd;
    for (uint32_t i=0;i<ueNum;i++)
    {
        v_modelUl[i] = CreateObject<MyModel>();
        v_modelUl[i]->Setup(ueNetDev.Get(i), enbNetDev.Get(0)->GetAddress(),nPackets, simTime, isMobile[i], rd());
        Simulator::Schedule(initialSetupTime + MicroSeconds(100 * i), &MyModel::SendPacketUl, v_modelUl[i]);
    }

    nrHelper->EnableTraces();
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}