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
#include "ns3/energy-source.h"
#include "ns3/li-ion-energy-source.h"
#include "ns3/basic-energy-harvester.h"
#include "ns3/device-energy-model.h"
#include "ns3/wifi-radio-energy-model.h"
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
#include "ns3/emlsr-manager.h"
#include "ns3/packet-socket-server.h"
#include "ns3/qos-utils.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/wifi-phy-rx-trace-helper.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/wifi-utils.h"
#include "ns3/yans-wifi-helper.h"

#include <array>
#include <cmath>

#define PI 3.1415926535

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("single-bss-mld");


//Energy management


enum TrafficTypeEnum
{
    TRAFFIC_DETERMINISTIC,
    TRAFFIC_BERNOULLI,
    TRAFFIC_INVALID
};

// Per STA traffic config
struct TrafficConfig
{
    WifiDirection m_dir;
    TrafficTypeEnum m_type;
    AcIndex m_link1Ac;
    AcIndex m_link2Ac;
    double m_lambda;
    double m_determIntervalNs;
    bool m_split;
    double m_prob;
};

using TrafficConfigMap = std::map<uint32_t /* Node ID */, TrafficConfig>;

Time slotTime;

WifiPhyRxTraceHelper wifiStats;

void
CheckStats()
{
    wifiStats.PrintStatistics();

    std::ofstream outFile("tx-timeline.txt");
    outFile << "Start Time,End Time,Source Node,DropReason\n";

    for (const auto& record : wifiStats.GetPpduRecords())
    {
        if (record.m_reason)
        {
            outFile << record.m_startTime.GetMilliSeconds()
                << "," // Convert Time to a numerical format
                << record.m_endTime.GetMilliSeconds()
                << "," // Convert Time to a numerical format
                << record.m_senderId << "," << record.m_reason << "\n";
        }
        else
        {
            bool allSuccess = true;
            for (const auto& status : record.m_statusPerMpdu)
            {
                if (!status)
                {
                    allSuccess = false;
                }
            }
            if (allSuccess)
            {
                outFile << record.m_startTime.GetMilliSeconds()
                    << "," // Convert Time to a numerical format
                    << record.m_endTime.GetMilliSeconds()
                    << "," // Convert Time to a numerical format
                    << record.m_senderId << ",success\n";
            }
            else
            {
                outFile << record.m_startTime.GetMilliSeconds()
                    << "," // Convert Time to a numerical format
                    << record.m_endTime.GetMilliSeconds()
                    << "," // Convert Time to a numerical format
                    << record.m_senderId << ",PayloadDecodeError\n";
            }
        }
    }
    outFile.close();
}

Ptr<PacketSocketClient>
GetDeterministicClient(const PacketSocketAddress& sockAddr,
                       const std::size_t pktSize,
                       const Time& interval,
                       const Time& start,
                       const AcIndex link1Ac,
                       const bool optionalTid = false,
                       const AcIndex link2Ac = AC_UNDEF,
                       const double optionalPr = 0)
{
    NS_ASSERT(link1Ac != AC_UNDEF);
    const auto link1Tids = wifiAcList.at(link1Ac);
    auto lowTid = link1Tids.GetLowTid();

    auto client = CreateObject<PacketSocketClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("MaxPackets", UintegerValue(0));
    client->SetAttribute("Interval", TimeValue(interval));
    client->SetAttribute("Priority", UintegerValue(lowTid));
    if (optionalTid && link2Ac != AC_UNDEF)
    {
        const auto link2Tids = wifiAcList.at(link2Ac);
        auto highTid = link2Tids.GetHighTid();
        client->SetAttribute("OptionalTid", UintegerValue(highTid));
        client->SetAttribute("OptionalTidPr", DoubleValue(optionalPr));
    }
    else
    {
        client->SetAttribute("OptionalTid", UintegerValue(lowTid));
    }
    client->SetRemote(sockAddr);
    client->SetStartTime(start);
    return client;
}

Ptr<BernoulliPacketSocketClient>
GetBernoulliClient(const PacketSocketAddress& sockAddr,
                   const std::size_t pktSize,
                   const double prob,
                   const Time& start,
                   const AcIndex link1Ac,
                   const bool optionalTid = false,
                   const AcIndex link2Ac = AC_UNDEF,
                   const double optionalPr = 0)
{
    NS_ASSERT(link1Ac != AC_UNDEF);
    const auto link1Tids = wifiAcList.at(link1Ac);
    auto lowTid = link1Tids.GetLowTid();

    auto client = CreateObject<BernoulliPacketSocketClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("MaxPackets", UintegerValue(0));
    client->SetAttribute("TimeSlot", TimeValue(slotTime));
    client->SetAttribute("BernoulliPr", DoubleValue(prob));
    client->SetAttribute("Priority", UintegerValue(lowTid));
    if (optionalTid && link2Ac != AC_UNDEF)
    {
        const auto link2Tids = wifiAcList.at(link2Ac);
        auto highTid = link2Tids.GetHighTid();
        client->SetAttribute("OptionalTid", UintegerValue(highTid));
        client->SetAttribute("OptionalTidPr", DoubleValue(optionalPr));
    }
    else
    {
        client->SetAttribute("OptionalTid", UintegerValue(lowTid));
    }
    client->SetRemote(sockAddr);
    client->SetStartTime(start);
    return client;
}

int
main(int argc, char* argv[])
{
    std::ofstream g_fileSummary;
    g_fileSummary.open("wifi-mld.dat", std::ofstream::app);
    bool printTxStats{false};
    bool printTxStatsSingleLine{true};
    bool printRxStats{false};

    // Will not change
    bool unlimitedAmpdu{true};
    uint8_t maxMpdusInAmpdu = 0;
    bool useRts{false};
    double bssRadius{0.001};
    double frequency{5};
    double frequency2{6};
    int gi = 800;
    double apTxPower = 20;
    double staTxPower = 20;

    // Input params
    uint32_t rngRun{6};
    double simulationTime{10}; // seconds
    uint32_t payloadSize = 1500;
    int mcs{6};
    int mcs2{6};
    int channelWidth = 20;
    int channelWidth2 = 20;
    // MLD STAs
    std::size_t nMldSta{5};
    double mldPerNodeLambda{0.00001};
    double mldProbLink1{0.5}; // prob_link1 + prob_link2 = 1
    uint8_t mldAcLink1Int{AC_BE};
    uint8_t mldAcLink2Int{AC_BE};
    // EDCA configuration for CWmins, CWmaxs
    uint64_t acBECwminLink1{16};
    uint8_t acBECwStageLink1{6};
    uint64_t acBECwminLink2{16};
    uint8_t acBECwStageLink2{6};
    uint64_t acBKCwminLink1{16};
    uint8_t acBKCwStageLink1{6};
    uint64_t acBKCwminLink2{16};
    uint8_t acBKCwStageLink2{6};
    uint64_t acVICwminLink1{16};
    uint8_t acVICwStageLink1{6};
    uint64_t acVICwminLink2{16};
    uint8_t acVICwStageLink2{6};
    uint64_t acVOCwminLink1{16};
    uint8_t acVOCwStageLink1{6};
    uint64_t acVOCwminLink2{16};
    uint8_t acVOCwStageLink2{6};
    bool enable_emlsr {true};

    CommandLine cmd(__FILE__);
    cmd.AddValue("rngRun", "Seed for simulation", rngRun);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("payloadSize", "Application payload size in Bytes", payloadSize);
    cmd.AddValue("enable_emlsr", "emlsr", enable_emlsr);
    cmd.AddValue("mcs", "MCS for link 1", mcs);
    cmd.AddValue("mcs2", "MCS for link 2", mcs2);
    cmd.AddValue("channelWidth", "Bandwidth for link 1", channelWidth);
    cmd.AddValue("channelWidth2", "Bandwidth for link 2", channelWidth2);
    cmd.AddValue("nMldSta", "Number of MLD STAs", nMldSta);
    cmd.AddValue("mldPerNodeLambda", "Per node arrival rate of MLD STAs", mldPerNodeLambda);
    cmd.AddValue("mldProbLink1", "MLD's splitting probability on link 1", mldProbLink1);
    cmd.AddValue("mldAcLink1Int", "AC of MLD", mldAcLink1Int);
    cmd.AddValue("mldAcLink2Int", "AC of MLD", mldAcLink2Int);
    cmd.AddValue("acBECwminLink1", "Initial CW for AC_BE", acBECwminLink1);
    cmd.AddValue("acBECwStageLink1", "Cutoff Stage for AC_BE", acBECwStageLink1);
    cmd.AddValue("acBKCwminLink1", "Initial CW for AC_BK", acBKCwminLink1);
    cmd.AddValue("acBKCwStageLink1", "Cutoff Stage for AC_BK", acBKCwStageLink1);
    cmd.AddValue("acVICwminLink1", "Initial CW for AC_VI", acVICwminLink1);
    cmd.AddValue("acVICwStageLink1", "Cutoff Stage for AC_VI", acVICwStageLink1);
    cmd.AddValue("acVOCwminLink1", "Initial CW for AC_VO", acVOCwminLink1);
    cmd.AddValue("acVOCwStageLink1", "Cutoff Stage for AC_VO", acVOCwStageLink1);
    cmd.AddValue("acBECwminLink2", "Initial CW for AC_BE", acBECwminLink2);
    cmd.AddValue("acBECwStageLink2", "Cutoff Stage for AC_BE", acBECwStageLink2);
    cmd.AddValue("acBKCwminLink2", "Initial CW for AC_BK", acBKCwminLink2);
    cmd.AddValue("acBKCwStageLink2", "Cutoff Stage for AC_BK", acBKCwStageLink2);
    cmd.AddValue("acVICwminLink2", "Initial CW for AC_VI", acVICwminLink2);
    cmd.AddValue("acVICwStageLink2", "Cutoff Stage for AC_VI", acVICwStageLink2);
    cmd.AddValue("acVOCwminLink2", "Initial CW for AC_VO", acVOCwminLink2);
    cmd.AddValue("acVOCwStageLink2", "Cutoff Stage for AC_VO", acVOCwStageLink2);
    cmd.Parse(argc, argv);
    uint8_t nLinks = 0;

    RngSeedManager::SetSeed(rngRun);
    RngSeedManager::SetRun(rngRun);
    uint32_t randomStream = rngRun;

    auto mldAcLink1 = static_cast<AcIndex>(mldAcLink1Int);
    auto mldAcLink2 = static_cast<AcIndex>(mldAcLink2Int);

    uint64_t acBECwmaxLink1 = acBECwminLink1 * pow(2, acBECwStageLink1);
    acBECwmaxLink1 -= 1;
    acBECwminLink1 -= 1;
    uint64_t acBKCwmaxLink1 = acBKCwminLink1 * pow(2, acBKCwStageLink1);
    acBKCwmaxLink1 -= 1;
    acBKCwminLink1 -= 1;
    uint64_t acVICwmaxLink1 = acVICwminLink1 * pow(2, acVICwStageLink1);
    acVICwmaxLink1 -= 1;
    acVICwminLink1 -= 1;
    uint64_t acVOCwmaxLink1 = acVOCwminLink1 * pow(2, acVOCwStageLink1);
    acVOCwmaxLink1 -= 1;
    acVOCwminLink1 -= 1;
    uint64_t acBECwmaxLink2 = acBECwminLink2 * pow(2, acBECwStageLink2);
    acBECwmaxLink2 -= 1;
    acBECwminLink2 -= 1;
    uint64_t acBKCwmaxLink2 = acBKCwminLink2 * pow(2, acBKCwStageLink2);
    acBKCwmaxLink2 -= 1;
    acBKCwminLink2 -= 1;
    uint64_t acVICwmaxLink2 = acVICwminLink2 * pow(2, acVICwStageLink2);
    acVICwmaxLink2 -= 1;
    acVICwminLink2 -= 1;
    uint64_t acVOCwmaxLink2 = acVOCwminLink2 * pow(2, acVOCwStageLink2);
    acVOCwmaxLink2 -= 1;
    acVOCwminLink2 -= 1;

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
    NodeContainer mldNodeCon;
    apNodeCon.Create(1);
    uint32_t nStaTotal = nMldSta;
    mldNodeCon.Create(nStaTotal);

    NetDeviceContainer apDevCon;
    NetDeviceContainer mldDevCon;

    WifiHelper mldWifiHelp;
    mldWifiHelp.SetStandard(WIFI_STANDARD_80211be);
    if (enable_emlsr){
        mldWifiHelp.ConfigEhtOptions("EmlsrActivated", BooleanValue(enable_emlsr));
        mldWifiHelp.ConfigEhtOptions("TransitionTimeout", TimeValue(MicroSeconds(1024)));
        mldWifiHelp.ConfigEhtOptions("MediumSyncDuration", TimeValue(MicroSeconds(3200)));
        mldWifiHelp.ConfigEhtOptions("MsdOfdmEdThreshold", IntegerValue(-72));
        mldWifiHelp.ConfigEhtOptions("MsdMaxNTxops", UintegerValue(0));
    }

    // Get channel string for MLD STA
    std::array<std::string, 2> mldChannelStr;
    for (auto freq : {frequency, frequency2})
    {
        std::string widthStr = (nLinks == 0)
                                   ? std::to_string(channelWidth)
                                   : std::to_string(channelWidth2);
        auto linkMcs = (nLinks == 0) ? mcs : mcs2;
        std::string dataModeStr = "EhtMcs" + std::to_string(linkMcs);
        mldChannelStr[nLinks] = "{0, " + widthStr + ", ";
        if (freq == 6)
        {
            mldChannelStr[nLinks] += "BAND_6GHZ, 0}";
            mldWifiHelp.SetRemoteStationManager(nLinks,
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue("OfdmRate24Mbps"));
        }
        else if (freq == 5)
        {
            mldChannelStr[nLinks] += "BAND_5GHZ, 0}";
            mldWifiHelp.SetRemoteStationManager(nLinks,
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue("OfdmRate24Mbps"));
        }
        else
        {
            std::cerr << "Unsupported frequency for reference BSS\n" << std::endl;
            return 0;
        }
        nLinks++;
    }

    SpectrumWifiPhyHelper mldPhyHelp(nLinks);
    // mldPhyHelp.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    


    Ptr<MultiModelSpectrumChannel> phy5ghzSpectrumChannel = CreateObject<
        MultiModelSpectrumChannel>();
    // Reference Loss for Friss at 1 m with 5.15 GHz
    Ptr<LogDistancePropagationLossModel> phy5ghzLossModel =
        CreateObject<LogDistancePropagationLossModel>();
    phy5ghzLossModel->SetAttribute("Exponent", DoubleValue(3.5));
    phy5ghzLossModel->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    phy5ghzLossModel->SetAttribute("ReferenceLoss", DoubleValue(50));
    phy5ghzSpectrumChannel->AddPropagationLossModel(phy5ghzLossModel);

    Ptr<MultiModelSpectrumChannel> phy6ghzSpectrumChannel = CreateObject<
        MultiModelSpectrumChannel>();
    // Reference Loss for Friss at 1 m with 6.0 GHz
    Ptr<LogDistancePropagationLossModel> phy6ghzLossModel =
        CreateObject<LogDistancePropagationLossModel>();
    phy6ghzLossModel->SetAttribute("Exponent", DoubleValue(2.0));
    phy6ghzLossModel->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    phy6ghzLossModel->SetAttribute("ReferenceLoss", DoubleValue(49.013));
    phy6ghzSpectrumChannel->AddPropagationLossModel(phy6ghzLossModel);

    mldPhyHelp.Set("Antennas", UintegerValue(1)); // Number of antennas (e.g., 4)
    mldPhyHelp.Set("MaxSupportedTxSpatialStreams", UintegerValue(1)); // Maximum Tx streams
    mldPhyHelp.Set("MaxSupportedRxSpatialStreams", UintegerValue(1)); // Maximum Rx streams

    mldPhyHelp.AddChannel(phy5ghzSpectrumChannel, WIFI_SPECTRUM_5_GHZ);
    mldPhyHelp.AddChannel(phy6ghzSpectrumChannel, WIFI_SPECTRUM_6_GHZ);

    for (uint8_t linkId = 0; linkId < nLinks; linkId++)
    {
        mldPhyHelp.Set(linkId, "ChannelSettings", StringValue(mldChannelStr[linkId]));
    }

    WifiMacHelper mldMacHelp;
    Ssid bssSsid = Ssid("BSS-SLD-MLD-COEX");

    // Set up MLD STAs
    if(enable_emlsr)
    {
        mldMacHelp.SetType("ns3::StaWifiMac",
                       "MaxMissedBeacons",
                       UintegerValue(std::numeric_limits<uint32_t>::max()),
                       "Ssid",
                       SsidValue(bssSsid));
    }
    else
    {
        mldMacHelp.SetType("ns3::StaWifiMac",
                       "MaxMissedBeacons",
                       UintegerValue(std::numeric_limits<uint32_t>::max()),
                       "Ssid",
                       SsidValue(bssSsid));
    }
    if (enable_emlsr){
        mldMacHelp.SetEmlsrManager("ns3::DefaultEmlsrManager",
                        "EmlsrLinkSet",
                        StringValue("1,2"),  // enable EMLSR on 5,6 Ghz links
                        "MainPhyId",
                        UintegerValue(1),
                        "EmlsrPaddingDelay",
                        TimeValue(MicroSeconds(32)),
                        "EmlsrTransitionDelay",
                        TimeValue(MicroSeconds(128)),
                        "SwitchAuxPhy",
                        BooleanValue(false),
                        "AuxPhyChannelWidth",
                        UintegerValue(40));                                                                                                                 
    }

    mldPhyHelp.Set("TxPowerStart", DoubleValue(staTxPower));
    mldPhyHelp.Set("TxPowerEnd", DoubleValue(staTxPower));
    mldDevCon = mldWifiHelp.Install(mldPhyHelp, mldMacHelp, mldNodeCon);

    uint64_t beaconInterval = std::min<uint64_t>(
        (ceil((simulationTime * 1000000) / 1024) * 1024),
        (65535 * 1024)); // beacon interval needs to be a multiple of time units (1024 us)

    // Set up AP
    mldMacHelp.SetType("ns3::ApWifiMac",
                       "BeaconInterval",
                       TimeValue(MicroSeconds(beaconInterval)),
                       "EnableBeaconJitter",
                       BooleanValue(false),
                       "Ssid",
                       SsidValue(bssSsid));
    mldPhyHelp.Set("TxPowerStart", DoubleValue(apTxPower));
    mldPhyHelp.Set("TxPowerEnd", DoubleValue(apTxPower));
    apDevCon = mldWifiHelp.Install(mldPhyHelp, mldMacHelp, apNodeCon);

    NetDeviceContainer allNetDevices;
    allNetDevices.Add(apDevCon);
    allNetDevices.Add(mldDevCon);

    mldWifiHelp.AssignStreams(allNetDevices, randomStream);

    

    // Enable TID-to-Link Mapping for AP and MLD STAs
    for (auto i = allNetDevices.Begin(); i != allNetDevices.End(); ++i)
    {
        auto wifiDev = DynamicCast<WifiNetDevice>(*i);
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingNegSupport",
                                                               EnumValue(
                                                                   WifiTidToLinkMappingNegSupport::ANY_LINK_SET));
    }
//Upper MAC traffic allocation
    // Map all low TIDs to link 1 (whose linkId=0), all high TIDs to link 2 (whose linkId=1)
    // E.g. BE traffic with TIDs 0 and 3 are sents to Link 0 and 1, respectively.
    // Adding mapping information at MLD STAs side
    // NOTE: only consider UL data traffic for now
    std::string mldMappingStr = "0,1,4,6 0; 3,2,5,7 1";
    // To use greedy (ns-3 default way for traffic-to-link allocation):
    // (1) use the default string below
    // (2) need to also set mldProbLink1 to 0 or 1 to have only one L-MAC queue
    std::string mldMappingStrDefault = "0,1,2,3,4,5,6,7 0,1";
    for (auto i = mldDevCon.Begin(); i != mldDevCon.End(); ++i)
    {
        auto wifiDev = DynamicCast<WifiNetDevice>(*i);
        wifiDev->GetMac()->SetAttribute("ActiveProbing", BooleanValue(true));
        // wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingDl", StringValue(mldMappingStr));
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute(
            "TidToLinkMappingUl",
            StringValue(mldMappingStr));
    }

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

    // set cwmins and cwmaxs for all Access Categories on ALL devices
    // (incl. AP because STAs sync with AP via association, probe, and beacon)
    std::string prefixStr = "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/";
    std::list<uint64_t> acBeCwmins = {acBECwminLink1, acBECwminLink2};
    Config::Set(prefixStr + "BE_Txop/MinCws", AttributeContainerValue<UintegerValue>(acBeCwmins));
    std::list<uint64_t> acBeCwmaxs = {acBECwmaxLink1, acBECwmaxLink2};
    Config::Set(prefixStr + "BE_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acBeCwmaxs));
    std::list<uint64_t> acBkCwmins = {acBKCwminLink1, acBKCwminLink2};
    Config::Set(prefixStr + "BK_Txop/MinCws", AttributeContainerValue<UintegerValue>(acBkCwmins));
    std::list<uint64_t> acBkCwmaxs = {acBKCwmaxLink1, acBKCwmaxLink2};
    Config::Set(prefixStr + "BK_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acBkCwmaxs));
    std::list<uint64_t> acViCwmins = {acVICwminLink1, acVICwminLink2};
    Config::Set(prefixStr + "VI_Txop/MinCws", AttributeContainerValue<UintegerValue>(acViCwmins));
    std::list<uint64_t> acViCwmaxs = {acVICwmaxLink1, acVICwmaxLink2};
    Config::Set(prefixStr + "VI_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acViCwmaxs));
    std::list<uint64_t> acVoCwmins = {acVOCwminLink1, acVOCwminLink2};
    Config::Set(prefixStr + "VO_Txop/MinCws", AttributeContainerValue<UintegerValue>(acVoCwmins));
    std::list<uint64_t> acVoCwmaxs = {acVOCwmaxLink1, acVOCwmaxLink2};
    Config::Set(prefixStr + "VO_Txop/MaxCws", AttributeContainerValue<UintegerValue>(acVoCwmaxs));

    // set all aifsn to be 2 (so that all aifs equal to legacy difs)
    std::list<uint64_t> aifsnList = {2, 2};
    Config::Set(prefixStr + "BE_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsnList));
    Config::Set(prefixStr + "BK_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsnList));
    Config::Set(prefixStr + "VI_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsnList));
    Config::Set(prefixStr + "VO_Txop/Aifsns", AttributeContainerValue<UintegerValue>(aifsnList));
    std::list<Time> txopLimitList = {MicroSeconds(0), MicroSeconds(0)};
    Config::Set(prefixStr + "BE_Txop/TxopLimits",
                AttributeContainerValue<TimeValue>(txopLimitList));
    Config::Set(prefixStr + "BK_Txop/TxopLimits",
                AttributeContainerValue<TimeValue>(txopLimitList));
    Config::Set(prefixStr + "VI_Txop/TxopLimits",
                AttributeContainerValue<TimeValue>(txopLimitList));
    Config::Set(prefixStr + "VO_Txop/TxopLimits",
                AttributeContainerValue<TimeValue>(txopLimitList));

    auto staWifiManager =
        DynamicCast<ConstantRateWifiManager>(DynamicCast<WifiNetDevice>(mldDevCon.Get(0))
            ->GetRemoteStationManager());
    slotTime = staWifiManager->GetPhy()->GetSlot();
    auto sifsTime = staWifiManager->GetPhy()->GetSifs();
    auto difsTime = sifsTime + 2 * slotTime;

    // mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    double angle = (static_cast<double>(360) / nStaTotal);
    positionAlloc->Add(Vector(1.0, 1.0, 0.0));
    for (uint32_t i = 0; i < nStaTotal; ++i)
    {
        positionAlloc->Add(Vector(1.0 + (bssRadius * cos((i * angle * PI) / 180)),
                                  1.0 + (bssRadius * sin((i * angle * PI) / 180)),
                                  0.0));
    }

    mobility.SetPositionAllocator(positionAlloc);
    NodeContainer allNodeCon(apNodeCon, mldNodeCon);
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
    double mldDetermIntervalNs = slotTime.GetNanoSeconds() / mldPerNodeLambda;
    for (uint32_t i = 0; i < nStaTotal; ++i)
    {
        trafficConfigMap[i] = {WifiDirection::UPLINK, TRAFFIC_BERNOULLI, mldAcLink1, mldAcLink2,
                               mldPerNodeLambda, mldDetermIntervalNs,
                               true, 1 - mldProbLink1};
    }

    // next, setup clients according to the config
    for (uint32_t i = 0; i < nStaTotal; ++i)
    {
        auto mapIt = trafficConfigMap.find(i);
        Ptr<Node> clientNode = (mapIt->second.m_dir == WifiDirection::UPLINK)
                                   ? mldNodeCon.Get(i)
                                   : apNodeCon.Get(0);
        Ptr<WifiNetDevice> clientDevice = DynamicCast<WifiNetDevice>(clientNode->GetDevice(0));
        Ptr<Node> serverNode = (mapIt->second.m_dir == WifiDirection::UPLINK)
                                   ? apNodeCon.Get(0)
                                   : mldNodeCon.Get(i);
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
                                                              mapIt->second.m_link1Ac,
                                                              mapIt->second.m_split,
                                                              mapIt->second.m_link2Ac,
                                                              mapIt->second.m_prob));
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
                                                          mapIt->second.m_link1Ac,
                                                          mapIt->second.m_split,
                                                          mapIt->second.m_link2Ac,
                                                          mapIt->second.m_prob));
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

    // RX stats
    wifiStats.Enable(allNodeCon);
    wifiStats.Start(Seconds(5));
    wifiStats.Stop(Seconds(5 + simulationTime));
    if (printRxStats)
    {
        Simulator::Schedule(Seconds(5 + simulationTime), &CheckStats);
    }

    // mldPhyHelp.EnablePcap("single-bss-coex", allNetDevices);
    // AsciiTraceHelper asciiTrace;
    // mldPhyHelp.EnableAsciiAll(asciiTrace.CreateFileStream("single-bss-coex.tr"));

    Simulator::Stop(Seconds(5 + simulationTime));
    Simulator::Run();

    auto finalResults = wifiTxStats.GetStatistics();
    auto successInfo = wifiTxStats.GetSuccessInfoMap();

    // total and mean delay calculation per node and link
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double> > >
        enqueueTimeMap;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double> > >
        dequeueTimeMap;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double> > >
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
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double> >
        totalQueuingDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double> >
        meanQueuingDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double> >
        totalAccessDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, double> >
        meanAccessDelayPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double> > >
        accessDelaysPerNodeLink;
    std::map<uint32_t /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<double> > >
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

    if (printTxStats)
    {
        std::cout << "TX Stats:\n";
        std::cout << "Node_ID\tLink_ID\t#Success\n";
        for (const auto& nodeMap : finalResults.m_numSuccessPerNodeLink)
        {
            for (const auto& linkMap : nodeMap.second)
            {
                std::cout << nodeMap.first << "\t\t"
                    << +linkMap.first << "\t\t"
                    << linkMap.second << "\n";
            }
        }
        std::cout << "Node_ID\tLink_ID\tMean_Queuing_Delay\n";
        for (const auto& nodeMap : meanQueuingDelayPerNodeLink)
        {
            for (const auto& linkMap : nodeMap.second)
            {
                std::cout << nodeMap.first << "\t\t"
                    << +linkMap.first << "\t\t"
                    << linkMap.second << "\n";
            }
        }
        std::cout << "Node_ID\tLink_ID\tMean_Access_Delay\n";
        for (const auto& nodeMap : meanAccessDelayPerNodeLink)
        {
            for (const auto& linkMap : nodeMap.second)
            {
                std::cout << nodeMap.first << "\t\t"
                    << +linkMap.first << "\t\t"
                    << linkMap.second << "\n";
            }
        }
        std::cout << "Summary:"
            << "\n1. Successful pkts: " << finalResults.m_numSuccess
            << "\n2. Successful and retransmitted pkts: " << finalResults.m_numRetransmitted
            << "\n3. Avg retransmissions per successful pkt: " << finalResults.m_avgFailures
            << "\n4. Failed pkts: " << finalResults.m_numFinalFailed
            << "\n";
    }

    // MLD's per link and total successful tx pr
    std::map<uint8_t /* Link ID */, uint64_t> numMldSuccessPerLink;
    std::map<uint8_t /* Link ID */, uint64_t> numMldAttemptsPerLink;
    uint64_t numMldSuccessTotal{0};
    uint64_t numMldAttemptsTotal{0};
    for (uint32_t i = 1; i < 1 + nMldSta; ++i)
    {
        const auto& linkMap = successInfo[i];
        for (const auto& records : linkMap)
        {
            for (const auto& pkt : records.second)
            {
                numMldSuccessPerLink[records.first] += 1;
                numMldAttemptsPerLink[records.first] += 1 + pkt.m_failures;
                numMldSuccessTotal += 1;
                numMldAttemptsTotal += 1 + pkt.m_failures;
            }
        }
    }
    double mldSuccPrTotal = static_cast<long double>(numMldSuccessTotal) / numMldAttemptsTotal;
    double mldSuccPrLink1 = static_cast<long double>(numMldSuccessPerLink[0]) /
                            numMldAttemptsPerLink[0];
    double mldSuccPrLink2 = static_cast<long double>(numMldSuccessPerLink[1]) /
                            numMldAttemptsPerLink[1];

    // throughput of MLD
    double mldThptTotal = static_cast<long double>(numMldSuccessTotal) * payloadSize * 8 /
                          simulationTime /
                          1000000;
    double mldThptLink1 = static_cast<long double>(numMldSuccessPerLink[0]) * payloadSize * 8 /
                          simulationTime /
                          1000000;
    double mldThptLink2 = static_cast<long double>(numMldSuccessPerLink[1]) * payloadSize * 8 /
                          simulationTime /
                          1000000;

    // mean delays of MLD
    std::map<uint8_t /* Link ID */, long double> mldQueDelayPerLinkTotal;
    long double mldQueDelayTotal{0};
    std::map<uint8_t /* Link ID */, long double> mldAccDelayPerLinkTotal;
    long double mldAccDelayTotal{0};
    for (uint32_t i = 1; i < 1 + nMldSta; ++i)
    {
        const auto& queLinkMap = totalQueuingDelayPerNodeLink[i];
        for (const auto& item : queLinkMap)
        {
            mldQueDelayPerLinkTotal[item.first] += item.second;
            mldQueDelayTotal += item.second;
        }
        const auto& accLinkMap = totalAccessDelayPerNodeLink[i];
        for (const auto& item : accLinkMap)
        {
            mldAccDelayPerLinkTotal[item.first] += item.second;
            mldAccDelayTotal += item.second;
        }
    }
    double mldMeanQueDelayTotal = mldQueDelayTotal / numMldSuccessTotal;
    double mldMeanQueDelayLink1 = mldQueDelayPerLinkTotal[0] / numMldSuccessPerLink[0];
    double mldMeanQueDelayLink2 = mldQueDelayPerLinkTotal[1] / numMldSuccessPerLink[1];
    double mldMeanAccDelayTotal = mldAccDelayTotal / numMldSuccessTotal;
    double mldMeanAccDelayLink1 = mldAccDelayPerLinkTotal[0] / numMldSuccessPerLink[0];
    double mldMeanAccDelayLink2 = mldAccDelayPerLinkTotal[1] / numMldSuccessPerLink[1];
    // Second raw moment of access delay: mean of (D_a)^2
    // Second central moment (variance) of access delay: mean of (D_a - mean)^2
    std::map<uint8_t /* Link ID */, long double> mldAccDelaySquarePerLinkTotal;
    long double mldAccDelaySquareTotal{0};
    std::map<uint8_t /* Link ID */, long double> mldAccDelayCentralSquarePerLinkTotal;
    long double mldAccDelayCentralSquareTotal{0};
    for (uint32_t i = 1; i < 1 + nMldSta; ++i)
    {
        const auto& accLinkMap = accessDelaysPerNodeLink[i];
        for (const auto& linkAccVec : accLinkMap)
        {
            const auto& accVec = linkAccVec.second;
            auto mldMeanAccDelayLink = (linkAccVec.first == 0)
                                           ? mldMeanAccDelayLink1
                                           : mldMeanAccDelayLink2;
            for (const auto& item : accVec)
            {
                mldAccDelaySquarePerLinkTotal[linkAccVec.first] += item * item;
                mldAccDelaySquareTotal += item * item;
                mldAccDelayCentralSquarePerLinkTotal[linkAccVec.first] +=
                    (item - mldMeanAccDelayLink) * (item - mldMeanAccDelayLink);
                mldAccDelayCentralSquareTotal +=
                    (item - mldMeanAccDelayLink) * (item - mldMeanAccDelayLink);
            }
        }
    }
    double mldSecondRawMomentAccDelayTotal = mldAccDelaySquareTotal / numMldSuccessTotal;
    double mldSecondRawMomentAccDelayLink1 =
        mldAccDelaySquarePerLinkTotal[0] / numMldSuccessPerLink[0];
    double mldSecondRawMomentAccDelayLink2 =
        mldAccDelaySquarePerLinkTotal[1] / numMldSuccessPerLink[1];
    double mldSecondCentralMomentAccDelayTotal = mldAccDelayCentralSquareTotal / numMldSuccessTotal;
    double mldSecondCentralMomentAccDelayLink1 =
        mldAccDelayCentralSquarePerLinkTotal[0] / numMldSuccessPerLink[0];
    double mldSecondCentralMomentAccDelayLink2 =
        mldAccDelayCentralSquarePerLinkTotal[1] / numMldSuccessPerLink[1];
    double mldMeanE2eDelayTotal = mldMeanQueDelayTotal + mldMeanAccDelayTotal;
    double mldMeanE2eDelayLink1 = mldMeanQueDelayLink1 + mldMeanAccDelayLink1;
    double mldMeanE2eDelayLink2 = mldMeanQueDelayLink2 + mldMeanAccDelayLink2;

    if (printTxStatsSingleLine)
    {
        g_fileSummary
            << mldSuccPrLink1 << "," << mldSuccPrLink2 << "," << mldSuccPrTotal << ","
            << mldThptLink1 << "," << mldThptLink2 << "," << mldThptTotal << ","
            << mldMeanQueDelayLink1 << "," << mldMeanQueDelayLink2 << "," << mldMeanQueDelayTotal <<
            ","
            << mldMeanAccDelayLink1 << "," << mldMeanAccDelayLink2 << "," << mldMeanAccDelayTotal <<
            ","
            << mldMeanE2eDelayLink1 << "," << mldMeanE2eDelayLink2 << "," << mldMeanE2eDelayTotal <<
            ",";
        // added jitter (second moment, raw/central) results (10 columns)
        g_fileSummary << mldSecondRawMomentAccDelayLink1 << "," << mldSecondRawMomentAccDelayLink2
            << "," << mldSecondRawMomentAccDelayTotal << ","
            << mldSecondCentralMomentAccDelayLink1 << "," << mldSecondCentralMomentAccDelayLink2
            << "," << mldSecondCentralMomentAccDelayTotal << ",";

        // print these input:
        g_fileSummary << rngRun << "," << simulationTime << "," << payloadSize << ","
            << mcs << "," << mcs2 << "," << channelWidth << "," << channelWidth2 << ","
            << nMldSta << "," << mldPerNodeLambda << "," << mldProbLink1 << ","
            << +mldAcLink1Int << "," << +mldAcLink2Int << ","
            << acBECwminLink1 << "," << +acBECwStageLink1 << ","
            << acBKCwminLink1 << "," << +acBKCwStageLink1 << ","
            << acVICwminLink1 << "," << +acVICwStageLink1 << ","
            << acVOCwminLink1 << "," << +acVOCwStageLink1 << ","
            << acBECwminLink2 << "," << +acBECwStageLink2 << ","
            << acBKCwminLink2 << "," << +acBKCwStageLink2 << ","
            << acVICwminLink2 << "," << +acVICwStageLink2 << ","
            << acVOCwminLink2 << "," << +acVOCwStageLink2 << "\n";
    }
    g_fileSummary.close();
    Simulator::Destroy();
    return 0;
}