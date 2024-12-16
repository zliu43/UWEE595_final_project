/*
 * Copyright (c) 2024
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/attribute-container.h"
#include "ns3/bernoulli_packet_socket_client.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-rate-wifi-manager.h"
#include "ns3/eht-configuration.h"
#include "ns3/eht-phy.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/qos-utils.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/wifi-utils.h"
#include "ns3/yans-wifi-helper.h"

#include <array>
#include <cmath>

#define PI 3.1415926535

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("single-bss-sld");

enum TrafficTypeEnum
{
    TRAFFIC_DETERMINISTIC,
    TRAFFIC_BERNOULLI,
    TRAFFIC_INVALID
};

// Per SLD traffic config
struct TrafficConfig
{
    WifiDirection m_dir;
    TrafficTypeEnum m_type;
    AcIndex m_linkAc;
    double m_lambda;
    double m_determIntervalNs;
};

using TrafficConfigMap = std::map<uint32_t /* Node ID */, TrafficConfig>;

Time slotTime;

Ptr<PacketSocketClient>
GetDeterministicClient(const PacketSocketAddress& sockAddr,
                       const std::size_t pktSize,
                       const Time& interval,
                       const Time& start,
                       const AcIndex linkAc)
{
    NS_ASSERT(linkAc != AC_UNDEF);
    auto tid = wifiAcList.at(linkAc).GetLowTid(); // use the low TID

    auto client = CreateObject<PacketSocketClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("MaxPackets", UintegerValue(0));
    client->SetAttribute("Interval", TimeValue(interval));
    client->SetAttribute("Priority", UintegerValue(tid));
    client->SetRemote(sockAddr);
    client->SetStartTime(start);
    return client;
}

Ptr<BernoulliPacketSocketClient>
GetBernoulliClient(const PacketSocketAddress& sockAddr,
                   const std::size_t pktSize,
                   const double prob,
                   const Time& start,
                   const AcIndex linkAc)
{
    NS_ASSERT(linkAc != AC_UNDEF);
    auto tid = wifiAcList.at(linkAc).GetLowTid();

    auto client = CreateObject<BernoulliPacketSocketClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("MaxPackets", UintegerValue(0));
    client->SetAttribute("TimeSlot", TimeValue(slotTime));
    client->SetAttribute("BernoulliPr", DoubleValue(prob));
    client->SetAttribute("Priority", UintegerValue(tid));
    client->SetRemote(sockAddr);
    client->SetStartTime(start);
    return client;
}

int
main(int argc, char* argv[])
{
    std::ofstream g_fileSummary;
    g_fileSummary.open("wifi-dcf.dat", std::ofstream::app);
    bool printTxStatsSingleLine{true};

    uint32_t rngRun{6};
    double simulationTime{20}; // seconds
    uint32_t payloadSize = 1500;
    double bssRadius{0.001};
    bool unlimitedAmpdu{false};
    uint8_t maxMpdusInAmpdu = 0;
    bool useRts{false};
    int gi = 800;
    double apTxPower = 20;
    double staTxPower = 20;

    // link parameters
    double frequency{5};
    int mcs{6};
    int channelWidth = 20;

    // SLD STAs parameters
    std::size_t nSld{5};
    double perSldLambda{0.00001};
    uint8_t sldAcInt{AC_BE}; // Access Category

    // EDCA configuration for CWmins, CWmaxs
    uint64_t acBECwmin{16};
    uint8_t acBECwStage{6};
    uint64_t acBKCwmin{16};
    uint8_t acBKCwStage{6};
    uint64_t acVICwmin{16};
    uint8_t acVICwStage{6};
    uint64_t acVOCwmin{16};
    uint8_t acVOCwStage{6};

    CommandLine cmd(__FILE__);
    cmd.AddValue("rngRun", "Seed for simulation", rngRun);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("payloadSize", "Application payload size in Bytes", payloadSize);
    cmd.AddValue("mcs", "MCS", mcs);
    cmd.AddValue("channelWidth", "Bandwidth", channelWidth);
    cmd.AddValue("nSld", "Number of SLD STAs on link 1", nSld);
    cmd.AddValue("perSldLambda",
                 "Per node Bernoulli arrival rate of SLD STAs",
                 perSldLambda);
    // cmd.AddValue("sldAcInt", "AC of SLD", sldAcInt);
    cmd.AddValue("acBECwmin", "Initial CW for AC_BE", acBECwmin);
    cmd.AddValue("acBECwStage", "Cutoff Stage for AC_BE", acBECwStage);
    // cmd.AddValue("acBKCwmin", "Initial CW for AC_BK", acBKCwmin);
    // cmd.AddValue("acBKCwStage", "Cutoff Stage for AC_BK", acBKCwStage);
    // cmd.AddValue("acVICwmin", "Initial CW for AC_VI", acVICwmin);
    // cmd.AddValue("acVICwStage", "Cutoff Stage for AC_VI", acVICwStage);
    // cmd.AddValue("acVOCwmin", "Initial CW for AC_VO", acVOCwmin);
    // cmd.AddValue("acVOCwStage", "Cutoff Stage for AC_VO", acVOCwStage);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(rngRun);
    RngSeedManager::SetRun(rngRun);
    uint32_t randomStream = rngRun;
    auto sldAc = static_cast<AcIndex>(sldAcInt);

    uint64_t acBECwmax = acBECwmin * pow(2, acBECwStage);
    acBECwmax -= 1;
    acBECwmin -= 1;
    uint64_t acBKCwmax = acBKCwmin * pow(2, acBKCwStage);
    acBKCwmax -= 1;
    acBKCwmin -= 1;
    uint64_t acVICwmax = acVICwmin * pow(2, acVICwStage);
    acVICwmax -= 1;
    acVICwmin -= 1;
    uint64_t acVOCwmax = acVOCwmin * pow(2, acVOCwStage);
    acVOCwmax -= 1;
    acVOCwmin -= 1;

    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    // Disable fragmentation
    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
                       UintegerValue(payloadSize + 100));

    // Make retransmissions persistent
    Config::SetDefault("ns3::WifiRemoteStationManager::MaxSlrc",
                       UintegerValue(std::numeric_limits<uint32_t>::max()));
    Config::SetDefault("ns3::WifiRemoteStationManager::MaxSsrc",
                       UintegerValue(std::numeric_limits<uint32_t>::max()));

    // Set infinitely long queue
    Config::SetDefault(
        "ns3::WifiMacQueue::MaxSize",
        QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, std::numeric_limits<uint32_t>::max())));

    // Don't drop MPDUs due to long stay in queue
    Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(Seconds(2 * simulationTime)));

    NodeContainer apNodeCon;
    NodeContainer staNodeCon;
    apNodeCon.Create(1);
    staNodeCon.Create(nSld);

    NetDeviceContainer apDevCon;
    NetDeviceContainer staDevCon;

    WifiHelper wifiHelp;
    wifiHelp.SetStandard(WIFI_STANDARD_80211be);

    SpectrumWifiPhyHelper phyHelp{};
    phyHelp.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    Ptr<MultiModelSpectrumChannel> phySpectrumChannel = CreateObject<
        MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel =
        CreateObject<LogDistancePropagationLossModel>();
    phySpectrumChannel->AddPropagationLossModel(lossModel);

    std::string dataModeStr = "EhtMcs" + std::to_string(mcs);
    wifiHelp.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     StringValue(dataModeStr));
    std::string channelStr = "{0, " + std::to_string(channelWidth) + ", ";
    if (frequency == 2.4)
    {
        channelStr += "BAND_2_4GHZ, 0}";
        phyHelp.AddChannel(phySpectrumChannel, WIFI_SPECTRUM_2_4_GHZ);
    }
    else if (frequency == 5)
    {
        channelStr += "BAND_5GHZ, 0}";
        phyHelp.AddChannel(phySpectrumChannel, WIFI_SPECTRUM_5_GHZ);
    }
    else if (frequency == 6)
    {
        channelStr += "BAND_6GHZ, 0}";
        phyHelp.AddChannel(phySpectrumChannel, WIFI_SPECTRUM_6_GHZ);
    }
    else
    {
        std::cout << "Unsupported frequency band!\n";
        return 0;
    }
    phyHelp.Set("ChannelSettings", StringValue(channelStr));

    WifiMacHelper macHelp;
    Ssid bssSsid = Ssid("BSS-SLD-ONLY");

    // Set up MLD STAs
    macHelp.SetType("ns3::StaWifiMac",
                    "MaxMissedBeacons",
                    UintegerValue(std::numeric_limits<uint32_t>::max()),
                    "Ssid",
                    SsidValue(bssSsid));
    phyHelp.Set("TxPowerStart", DoubleValue(staTxPower));
    phyHelp.Set("TxPowerEnd", DoubleValue(staTxPower));
    staDevCon = wifiHelp.Install(phyHelp, macHelp, staNodeCon);

    uint64_t beaconInterval = std::min<uint64_t>(
        (ceil((simulationTime * 1000000) / 1024) * 1024),
        (65535 * 1024)); // beacon interval needs to be a multiple of time units (1024 us)

    // Set up AP
    macHelp.SetType("ns3::ApWifiMac",
                    "BeaconInterval",
                    TimeValue(MicroSeconds(beaconInterval)),
                    "EnableBeaconJitter",
                    BooleanValue(false),
                    "Ssid",
                    SsidValue(bssSsid));
    phyHelp.Set("TxPowerStart", DoubleValue(apTxPower));
    phyHelp.Set("TxPowerEnd", DoubleValue(apTxPower));
    apDevCon = wifiHelp.Install(phyHelp, macHelp, apNodeCon);

    NetDeviceContainer allNetDevices;
    allNetDevices.Add(apDevCon);
    allNetDevices.Add(staDevCon);

    WifiHelper::AssignStreams(allNetDevices, randomStream);

    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                TimeValue(NanoSeconds(gi)));

    if (!unlimitedAmpdu)
    {
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(maxMpdusInAmpdu * (payloadSize + 50)));
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BK_MaxAmpduSize",
                    UintegerValue(maxMpdusInAmpdu * (payloadSize + 50)));
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VO_MaxAmpduSize",
                    UintegerValue(maxMpdusInAmpdu * (payloadSize + 50)));
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_MaxAmpduSize",
                    UintegerValue(maxMpdusInAmpdu * (payloadSize + 50)));
    }

    // Set cwmins and cwmaxs for all Access Categories on both AP and STAs
    // (including AP because STAs sync with AP via association, probe, and beacon)
    std::string prefixStr = "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/";
    std::list<uint64_t> acBeCwmins = {acBECwmin};
    std::list<uint64_t> acBeCwmaxs = {acBECwmax};
    std::list<uint64_t> acBkCwmins = {acBKCwmin};
    std::list<uint64_t> acBkCwmaxs = {acBKCwmax};
    std::list<uint64_t> acViCwmins = {acVICwmin};
    std::list<uint64_t> acViCwmaxs = {acVICwmax};
    std::list<uint64_t> acVoCwmins = {acVOCwmin};
    std::list<uint64_t> acVoCwmaxs = {acVOCwmax};
    Config::Set(prefixStr + "BE_Txop/MinCws", AttributeContainerValue<UintegerValue>(acBeCwmins));
    Config::Set(prefixStr + "BE_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acBeCwmaxs));
    Config::Set(prefixStr + "BK_Txop/MinCws", AttributeContainerValue<UintegerValue>(acBkCwmins));
    Config::Set(prefixStr + "BK_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acBkCwmaxs));
    Config::Set(prefixStr + "VI_Txop/MinCws", AttributeContainerValue<UintegerValue>(acViCwmins));
    Config::Set(prefixStr + "VI_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acViCwmaxs));
    Config::Set(prefixStr + "VO_Txop/MinCws", AttributeContainerValue<UintegerValue>(acVoCwmins));
    Config::Set(prefixStr + "VO_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acVoCwmaxs));

    // Set all aifsn to 2 (so that all AIFS equal to legacy DIFS)
    std::list<uint64_t> aifsns = {2};
    Config::Set(prefixStr + "BE_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsns));
    Config::Set(prefixStr + "BK_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsns));
    Config::Set(prefixStr + "VI_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsns));
    Config::Set(prefixStr + "VO_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsns));

    // Set all TXOP limit to 0
    std::list<Time> txopLimits = {MicroSeconds(0)};
    Config::Set(prefixStr + "BE_Txop/TxopLimits", AttributeContainerValue<TimeValue>(txopLimits));
    Config::Set(prefixStr + "BK_Txop/TxopLimits", AttributeContainerValue<TimeValue>(txopLimits));
    Config::Set(prefixStr + "VI_Txop/TxopLimits", AttributeContainerValue<TimeValue>(txopLimits));
    Config::Set(prefixStr + "VO_Txop/TxopLimits", AttributeContainerValue<TimeValue>(txopLimits));

    auto staWifiManager =
        DynamicCast<ConstantRateWifiManager>(DynamicCast<WifiNetDevice>(staDevCon.Get(0))
            ->GetRemoteStationManager());
    slotTime = staWifiManager->GetPhy()->GetSlot();
    auto sifsTime = staWifiManager->GetPhy()->GetSifs();
    auto difsTime = sifsTime + 2 * slotTime;

    // mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    double angle = (static_cast<double>(360) / nSld);
    positionAlloc->Add(Vector(1.0, 1.0, 0.0));
    for (uint32_t i = 0; i < nSld; ++i)
    {
        positionAlloc->Add(Vector(1.0 + (bssRadius * cos((i * angle * PI) / 180)),
                                  1.0 + (bssRadius * sin((i * angle * PI) / 180)),
                                  0.0));
    }
    mobility.SetPositionAllocator(positionAlloc);
    NodeContainer allNodeCon(apNodeCon, staNodeCon);
    mobility.Install(allNodeCon);

    /* Setting applications */
    // random start time
    Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable>();
    startTime->SetAttribute("Stream", IntegerValue(randomStream));
    startTime->SetAttribute("Min", DoubleValue(0.0));
    startTime->SetAttribute("Max", DoubleValue(1.0));

    // setup PacketSocketServer for every node
    PacketSocketHelper packetSocket;
    packetSocket.Install(allNodeCon);
    for (auto nodeIt = allNodeCon.Begin(); nodeIt != allNodeCon.End(); ++nodeIt)
    {
        PacketSocketAddress srvAddr;
        auto device = DynamicCast<WifiNetDevice>((*nodeIt)->GetDevice(0));
        srvAddr.SetSingleDevice(device->GetIfIndex());
        srvAddr.SetProtocol(1);
        auto psServer = CreateObject<PacketSocketServer>();
        psServer->SetLocal(srvAddr);
        (*nodeIt)->AddApplication(psServer);
        psServer->SetStartTime(Seconds(0)); // all servers start at 0 s
    }

    // set the configuration pairs for applications (UL, Bernoulli arrival)
    TrafficConfigMap trafficConfigMap;
    double sldDetermIntervalNs = slotTime.GetNanoSeconds() / perSldLambda;
    for (uint32_t i = 0; i < nSld; ++i)
    {
        trafficConfigMap[i] = {WifiDirection::UPLINK, TRAFFIC_BERNOULLI, sldAc,
                               perSldLambda, sldDetermIntervalNs};
    }

    // next, setup clients according to the config
    for (uint32_t i = 0; i < nSld; ++i)
    {
        auto mapIt = trafficConfigMap.find(i);
        Ptr<Node> clientNode = (mapIt->second.m_dir == WifiDirection::UPLINK)
                                   ? staNodeCon.Get(i)
                                   : apNodeCon.Get(0);
        Ptr<WifiNetDevice> clientDevice = DynamicCast<WifiNetDevice>(clientNode->GetDevice(0));
        Ptr<Node> serverNode = (mapIt->second.m_dir == WifiDirection::UPLINK)
                                   ? apNodeCon.Get(0)
                                   : staNodeCon.Get(i);
        Ptr<WifiNetDevice> serverDevice = DynamicCast<WifiNetDevice>(serverNode->GetDevice(0));

        switch (mapIt->second.m_type)
        {
        case TRAFFIC_DETERMINISTIC: {
            PacketSocketAddress sockAddr;
            sockAddr.SetSingleDevice(clientDevice->GetIfIndex());
            sockAddr.SetPhysicalAddress(serverDevice->GetAddress());
            sockAddr.SetProtocol(1);
            clientNode->AddApplication(GetDeterministicClient(sockAddr,
                                                              payloadSize,
                                                              NanoSeconds(
                                                                  mapIt->second.m_determIntervalNs),
                                                              Seconds(startTime->GetValue()),
                                                              mapIt->second.m_linkAc));
            break;
        }
        case TRAFFIC_BERNOULLI: {
            PacketSocketAddress sockAddr;
            sockAddr.SetSingleDevice(clientDevice->GetIfIndex());
            sockAddr.SetPhysicalAddress(serverDevice->GetAddress());
            sockAddr.SetProtocol(1);
            clientNode->AddApplication(GetBernoulliClient(sockAddr,
                                                          payloadSize,
                                                          mapIt->second.m_lambda,
                                                          Seconds(startTime->GetValue()),
                                                          mapIt->second.m_linkAc));
            break;
        }
        default: {
            std::cerr << "traffic type " << mapIt->second.m_type << " not supported\n";
            break;
        }
        }
    }

    // TX stats
    WifiTxStatsHelper wifiTxStats;
    wifiTxStats.Enable(allNetDevices);
    wifiTxStats.Start(Seconds(5));
    wifiTxStats.Stop(Seconds(5 + simulationTime));

    // phyHelp.EnablePcap("single-bss-sld", allNetDevices);
    // AsciiTraceHelper asciiTrace;
    // phyHelp.EnableAsciiAll(asciiTrace.CreateFileStream("single-bss-sld.tr"));

    Simulator::Stop(Seconds(5 + simulationTime));
    Simulator::Run();

    auto finalResults = wifiTxStats.GetStatistics();
    auto successInfo = wifiTxStats.GetSuccessInfoMap();

    // total and mean delay calculation per node and link
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double>>>
        enqueueTimeMap;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double>>>
        dequeueTimeMap;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double>>>
        holTimeMap;
    for (const auto& nodeMap : successInfo)
    {
        for (const auto& linkMap : nodeMap.second)
        {
            for (const auto& record : linkMap.second)
            {
                enqueueTimeMap[nodeMap.first][linkMap.first].emplace_back(record.m_enqueueMs);
                dequeueTimeMap[nodeMap.first][linkMap.first].emplace_back(record.m_dequeueMs);
            }
            for (uint32_t i = 0; i < enqueueTimeMap[nodeMap.first][linkMap.first].size(); ++i)
            {
                if (i == 0)
                {
                    // This value is false (some data packet may be already in queue
                    // because our stats did not start at 0 second), and will be removed later
                    holTimeMap[nodeMap.first][linkMap.first].emplace_back(
                        enqueueTimeMap[nodeMap.first][linkMap.first][i]);
                }
                else
                {
                    holTimeMap[nodeMap.first][linkMap.first].emplace_back(
                        std::max(enqueueTimeMap[nodeMap.first][linkMap.first][i],
                                 dequeueTimeMap[nodeMap.first][linkMap.first][i - 1]));
                }
            }
            // remove the first element
            enqueueTimeMap[nodeMap.first][linkMap.first].erase(
                enqueueTimeMap[nodeMap.first][linkMap.first].begin());
            dequeueTimeMap[nodeMap.first][linkMap.first].erase(
                dequeueTimeMap[nodeMap.first][linkMap.first].begin());
            holTimeMap[nodeMap.first][linkMap.first].erase(
                holTimeMap[nodeMap.first][linkMap.first].begin());
        }
    }
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double>>
        totalQueuingDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double>>
        meanQueuingDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double>>
        totalAccessDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double>>
        meanAccessDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double>>>
        accessDelaysPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double>>>
        e2eDelaysPerNodeLink;
    for (const auto& nodeMap : successInfo)
    {
        for (const auto& linkMap : nodeMap.second)
        {
            for (uint32_t i = 0; i < enqueueTimeMap[nodeMap.first][linkMap.first].size(); ++i)
            {
                totalQueuingDelayPerNodeLink[nodeMap.first][linkMap.first] += holTimeMap[nodeMap.
                    first][linkMap.first][i] - enqueueTimeMap[nodeMap.first][
                    linkMap.first][i];
                totalAccessDelayPerNodeLink[nodeMap.first][linkMap.first] += dequeueTimeMap[nodeMap.
                    first][linkMap.first][i] - holTimeMap[nodeMap.first][linkMap.
                    first][i];
                accessDelaysPerNodeLink[nodeMap.first][linkMap.first].emplace_back(
                    dequeueTimeMap[nodeMap.first][linkMap.first][i]
                    - holTimeMap[nodeMap.first][linkMap.first][i]);
                e2eDelaysPerNodeLink[nodeMap.first][linkMap.first].emplace_back(
                    dequeueTimeMap[nodeMap.first][linkMap.first][i]
                    - enqueueTimeMap[nodeMap.first][linkMap.first][i]);
            }
            meanQueuingDelayPerNodeLink[nodeMap.first][linkMap.first] =
                totalQueuingDelayPerNodeLink[nodeMap.first][linkMap.first] / (finalResults.
                    m_numSuccessPerNodeLink[nodeMap.first][linkMap.first] - 1);
            meanAccessDelayPerNodeLink[nodeMap.first][linkMap.first] =
                totalAccessDelayPerNodeLink[nodeMap.first][linkMap.first] / (finalResults.
                    m_numSuccessPerNodeLink[nodeMap.first][linkMap.first] - 1);
        }
    }

    // successful tx prob of SLD STAs
    uint64_t numSldSuccess = 0;
    uint64_t numSldAttempts = 0;
    for (uint32_t i = 1; i < 1 + nSld; ++i)
    {
        const auto& linkMap = successInfo[i];
        for (const auto& records : linkMap)
        {
            for (const auto& pkt : records.second)
            {
                numSldSuccess += 1;
                numSldAttempts += 1 + pkt.m_failures;
            }
        }
    }
    double sldSuccPr = static_cast<long double>(numSldSuccess) / numSldAttempts;

    // throughput of SLD STAs
    double sldThpt = static_cast<long double>(numSldSuccess) * payloadSize * 8 /
                     simulationTime /
                     1000000;

    // mean delays of SLD STAs
    long double sldQueDelayTotal = 0;
    long double sldAccDelayTotal = 0;
    for (uint32_t i = 1; i < 1 + nSld; ++i)
    {
        const auto& queLinkMap = totalQueuingDelayPerNodeLink[i];
        for (const auto& item : queLinkMap)
        {
            sldQueDelayTotal += item.second;
        }
        const auto accLinkMap = totalAccessDelayPerNodeLink[i];
        for (const auto& item : accLinkMap)
        {
            sldAccDelayTotal += item.second;
        }
    }
    double sldMeanQueDelay = sldQueDelayTotal / numSldSuccess;
    double sldMeanAccDelay = sldAccDelayTotal / numSldSuccess;

    double sldMeanE2eDelay = sldMeanQueDelay + sldMeanAccDelay;

    if (printTxStatsSingleLine)
    {
        g_fileSummary << sldSuccPr << ","
            << sldThpt << ","
            << sldMeanQueDelay << ","
            << sldMeanAccDelay << ","
            << sldMeanE2eDelay << ","
            << rngRun << ","
            << simulationTime << ","
            << payloadSize << ","
            << mcs << ","
            << channelWidth << ","
            << nSld << ","
            << perSldLambda << ","
            << +sldAcInt << ","
            << acBECwmin << ","
            << +acBECwStage << "\n";
    }
    g_fileSummary.close();
    Simulator::Destroy();
    return 0;
}