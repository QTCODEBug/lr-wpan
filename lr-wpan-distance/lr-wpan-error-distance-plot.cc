#include <ns3/abort.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/gnuplot.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-error-model.h>
#include <ns3/lr-wpan-mac.h>
#include <ns3/lr-wpan-net-device.h>
#include <ns3/lr-wpan-spectrum-value-helper.h>
#include <ns3/mac16-address.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/net-device.h>
#include <ns3/node.h>
#include <ns3/nstime.h>
#include <ns3/packet.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/spectrum-value.h>
#include <ns3/test.h>
#include <ns3/uinteger.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;
using namespace ns3::lrwpan;

uint32_t g_packetsReceived = 0; //!< Number of packets received

NS_LOG_COMPONENT_DEFINE("LrWpanErrorDistancePlot");

void LrWpanErrorDistanceCallback(McpsDataIndicationParams params, Ptr<Packet> p) {
    g_packetsReceived++;
}

int main(int argc, char* argv[]) {
    int minDistance = 1;
    int maxDistance = 100; // meters, default for testing
    int increment = 10; // distance increment
    int maxPackets = 1000;
    int packetSize = 7; 
    double txPower = 5; // Increased transmit power
    uint32_t channelNumber = 11;
    double rxSensitivity = -85.0; // Reduced sensitivity for testing

    CommandLine cmd(__FILE__);
    cmd.AddValue("txPower", "transmit power (dBm)", txPower);
    cmd.AddValue("packetSize", "packet (MSDU) size (bytes)", packetSize);
    cmd.AddValue("channelNumber", "channel number", channelNumber);
    cmd.AddValue("rxSensitivity", "receiver sensitivity (dBm)", rxSensitivity);
    cmd.AddValue("maxDistance", "maximum distance (m)", maxDistance); // Added for command-line control
    cmd.Parse(argc, argv);

    Gnuplot psrplot = Gnuplot("802.15.4-psr-distance.eps");
    Gnuplot2dDataset psrdataset("802.15.4-psr-vs-distance");

    // Create nodes and devices
    Ptr<Node> n0 = CreateObject<Node>();
    Ptr<Node> n1 = CreateObject<Node>();
    Ptr<Node> n2 = CreateObject<Node>();
    Ptr<Node> n3 = CreateObject<Node>();

    Ptr<LrWpanNetDevice> dev0 = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev1 = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev2 = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> dev3 = CreateObject<LrWpanNetDevice>();

    dev0->SetAddress(Mac16Address("00:01"));
    dev1->SetAddress(Mac16Address("00:02"));
    dev2->SetAddress(Mac16Address("00:03"));
    dev3->SetAddress(Mac16Address("00:04"));

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> model = CreateObject<LogDistancePropagationLossModel>();
    channel->AddPropagationLossModel(model);

    // Connect devices to the channel
    dev0->SetChannel(channel);
    dev1->SetChannel(channel);
    dev2->SetChannel(channel);
    dev3->SetChannel(channel);

    n0->AddDevice(dev0);
    n1->AddDevice(dev1);
    n2->AddDevice(dev2);
    n3->AddDevice(dev3);

    // Set up position models
    Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> mob2 = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> mob3 = CreateObject<ConstantPositionMobilityModel>();

    dev0->GetPhy()->SetMobility(mob0);
    dev1->GetPhy()->SetMobility(mob1);
    dev2->GetPhy()->SetMobility(mob2);
    dev3->GetPhy()->SetMobility(mob3);

    LrWpanSpectrumValueHelper svh;
    Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(txPower, channelNumber);
    dev0->GetPhy()->SetTxPowerSpectralDensity(psd);
    dev2->GetPhy()->SetTxPowerSpectralDensity(psd); 

    dev1->GetPhy()->SetRxSensitivity(rxSensitivity);
    dev3->GetPhy()->SetRxSensitivity(rxSensitivity); 

    // Set up data reception callback
    dev1->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&LrWpanErrorDistanceCallback));
    dev3->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&LrWpanErrorDistanceCallback));

    McpsDataRequestParams params;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstPanId = 0;
    params.m_msduHandle = 0;
    params.m_txOptions = 0;

    Ptr<Packet> p;
    mob0->SetPosition(Vector(0, 0, 0));
    mob1->SetPosition(Vector(minDistance, 0, 0));
    mob2->SetPosition(Vector(0, 10, 0));
    mob3->SetPosition(Vector(minDistance, 10, 0));

    // Start simulation
    for (int j = minDistance; j <= maxDistance; j += increment) {
        for (int i = 0; i < maxPackets; i++) {
            p = Create<Packet>(packetSize);
            params.m_dstAddr = Mac16Address("00:02");
            Simulator::Schedule(Seconds(i * 0.001), &LrWpanMac::McpsDataRequest, dev0->GetMac(), params, p);

            params.m_dstAddr = Mac16Address("00:04");
            Simulator::Schedule(Seconds(i * 0.001), &LrWpanMac::McpsDataRequest, dev2->GetMac(), params, p);
        }

        Simulator::Run();
        std::cout << "Distance: " << j << " m, Packets Received: " << g_packetsReceived << std::endl;
        psrdataset.Add(j, g_packetsReceived / (2.0 * maxPackets));
        g_packetsReceived = 0;

        mob1->SetPosition(Vector(j, 0, 0));
        mob3->SetPosition(Vector(j, 10, 0));
    }

    psrplot.AddDataset(psrdataset);
    psrplot.SetTitle("PSR between nodes with varying distance");
    psrplot.SetTerminal("postscript eps color enh \"Times-BoldItalic\"");
    psrplot.SetLegend("Distance (m)", "Packet Success Rate (PSR)");
    psrplot.SetExtra("set xrange [0:" + std::to_string(maxDistance) + "]\n\
                      set yrange [0:1]\n\
                      set grid\n\
                      set style line 1 linewidth 5\n\
                      set style increment user");
    std::ofstream berfile("802.15.4-psr-distance-4-nodes.plt");
    psrplot.GenerateOutput(berfile);
    berfile.close();

    Simulator::Destroy();
    return 0;
}
