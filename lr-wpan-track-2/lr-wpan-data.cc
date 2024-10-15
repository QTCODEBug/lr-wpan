#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/forwarder-helper.h>
#include <ns3/network-server-helper.h>
#include <iomanip>
#include <sstream>

using namespace ns3;
using namespace ns3::lrwpan;

// Forward declaration to use later for scheduling packet sending
void SendPacket(Ptr<NetworkServer> server, Ptr<Packet> pkt, Mac16Address destAddr);

// Function to log packet information
void AsciiTrace(Ptr<OutputStreamWrapper> stream, std::string context, Ptr<const Packet> packet) {
    if (stream && packet) {
        *stream->GetStream() << context << " Packet Size: " << packet->GetSize() << std::endl;
    }
}

static void DataIndication(McpsDataIndicationParams params, Ptr<Packet> p) {
    if (p) {
        NS_LOG_UNCOND("Received packet of size " << p->GetSize());
    } else {
        std::cerr << "Received packet is nullptr!" << std::endl;
    }
}

static void DataConfirm(McpsDataConfirmParams params) {
    NS_LOG_UNCOND("LrWpanMcpsDataConfirmStatus = " << params.m_status);
}

static void StateChangeNotification(std::string context, Time now, PhyEnumeration oldState, PhyEnumeration newState) {
    NS_LOG_UNCOND(context << " state change at " << now.As(Time::S) << " from "
                          << LrWpanHelper::LrWpanPhyEnumerationPrinter(oldState) << " to "
                          << LrWpanHelper::LrWpanPhyEnumerationPrinter(newState));
}

int main(int argc, char* argv[]) {
    bool verbose = false;
    CommandLine cmd;
    cmd.AddValue("verbose", "Turn on all log components", verbose);
    cmd.Parse(argc, argv);

    if (verbose) {
        LogComponentEnableAll(LOG_PREFIX_TIME | LOG_PREFIX_FUNC);
        LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
        LogComponentEnable("LrWpanMac", LOG_LEVEL_ALL);
    }

    // ASCII trace for packet information
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("lr-wpan-trace.tr");
    NS_ASSERT_MSG(stream != nullptr, "Failed to create ASCII trace file stream");

    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    NodeContainer nodes, gateways, networkServers;
    nodes.Create(8);
    gateways.Create(1);
    networkServers.Create(1);

    LrWpanHelper lrWpanHelper;
    NetDeviceContainer devices;

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<LrWpanNetDevice> dev = CreateObject<LrWpanNetDevice>();
        dev->SetChannel(channel);
        devices.Add(dev);
        nodes.Get(i)->AddDevice(dev);

        dev->SetAddress(Mac16Address(("00:0" + std::to_string(i + 1)).c_str()));

        dev->GetPhy()->TraceConnect("TrxState", "phy" + std::to_string(i), MakeCallback(&StateChangeNotification));
        dev->GetMac()->TraceConnectWithoutContext("MacTx", MakeBoundCallback(&AsciiTrace, stream, "MacTx - dev" + std::to_string(i)));

        dev->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&DataConfirm));
        dev->GetMac()->SetMcpsDataIndicationCallback(MakeCallback(&DataIndication));

        Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
        mobility->SetPosition(Vector(i * 10.0, 0, 0));
        dev->GetPhy()->SetMobility(mobility);
    }

    lrWpanHelper.EnablePcapAll("lr-wpan-all");

    // Set up the Network Server and Forwarder Helper
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGateways(gateways);
    networkServerHelper.SetEndDevices(nodes);
    ApplicationContainer apps = networkServerHelper.Install(networkServers);
    Ptr<NetworkServer> ns = DynamicCast<NetworkServer>(apps.Get(0));

    ForwarderHelper forwarderHelper;
    forwarderHelper.Install(gateways);

    // Schedule packet sending from Network Server
    Simulator::Schedule(Seconds(2.0), &SendPacket, ns, Create<Packet>(50), Mac16Address("00:02"));

    Simulator::Stop(Seconds(10));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}

// Function to send packet from Network Server
void SendPacket(Ptr<NetworkServer> server, Ptr<Packet> pkt, Mac16Address destAddr) {
    server->Send(pkt, destAddr);
}
