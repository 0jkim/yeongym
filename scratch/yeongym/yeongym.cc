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
#include "ns3/mobility-module.h"
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
    double bandwidthBand1 = 1e6;          // 1MHz 주파수 대역 (699.5 - 700.5MHz)
    uint16_t numerologyBwp1 = 0;          // mMTC numerology 0
    const uint16_t gNBNum = 1;            // 1 gNB
    const uint16_t ueNum = 100;           // 100 UE
    double isd = 1732.0;                  // ISD 1732m

    bool enableUl = true;     // Uplink on
    uint8_t sch = 1;          // 5G-Ofdma (Grant-based)
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
    cmd.AddValue("scheduler", "Scheduler Type", sch);
    cmd.AddValue("nPackets", "Number of paackets in each ue");
    cmd.AddValue("simTime", "The simulation time", simTime);
    cmd.Parse(argc, argv);

    /**
     * Configuration GridScenario
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

    // Configuration Mobility of UE
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
    mobility.Install(ueNodes);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");

    for (uint32_t i = 0; i < ueNum; i++)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            gridScenario.GetUserTerminals().Get(i)->GetObject<ConstantVelocityMobilityModel>();
        if (!mob)
        {
            NS_FATAL_ERROR("ConstantVelocityMobilityModel not found for UE " << i);
        }
        if (i < ueNum * 0.2)
        {
            mob->SetVelocity(Vector(27.28, 0, 0));
            NS_LOG_INFO("UE " << i << " set to 100km/h (car)");
        }
        else
        {
            mob->SetVelocity(Vector(0, 0, 0));
            NS_LOG_INFO("UE " << i << " set to 0km/h (Indoor)");
        }
    }

    // Configuration NR
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // Bandwidth environment for urban coverage for massive connection
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    1,
                                                    BandwidthPartInfo::UMa_nLoS);

    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    nrHelper->InitializeOperationBand(&band1);
    allBwps = CcBwpCreator::GetAllBwps({band1});

    // Install NetDevice
    NetDeviceContainer enbNetDev =
        nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AttachToClosestGnb(ueNetDev, enbNetDev);

    
}