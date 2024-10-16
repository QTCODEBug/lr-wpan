#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/ascii-file.h>
#include <iomanip>
#include <sstream>

using namespace ns3;
using namespace ns3::lrwpan;

void AsciiTrace(Ptr<OutputStreamWrapper> stream, std::string context, Ptr<const Packet> packet) {
    *stream->GetStream() << context << " Packet Size: " << packet->GetSize() << std::endl;
}

static void DataIndication(McpsDataIndicationParams params, Ptr<Packet> p) {
    NS_LOG_UNCOND("Received packet of size " << p->GetSize());
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
    bool extended = false;

    CommandLine cmd;
    cmd.AddValue("verbose", "turn on all log components", verbose);
    cmd.AddValue("extended", "use extended addressing", extended);
    cmd.Parse(argc, argv);

    if (verbose) {
        LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC));
        LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
        LogComponentEnable("LrWpanMac", LOG_LEVEL_ALL);
    }

    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("lr-wpan-trace.tr");
    NS_ASSERT_MSG(stream != nullptr, "Failed to create ASCII trace file stream");

    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    NodeContainer nodes;
    nodes.Create(4); // Sử dụng 4 nút

    LrWpanHelper lrWpanHelper;
    NetDeviceContainer devices;

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<LrWpanNetDevice> dev = CreateObject<LrWpanNetDevice>();
        dev->SetChannel(channel);
        devices.Add(dev);
        nodes.Get(i)->AddDevice(dev);

        if (!extended) {
            dev->SetAddress(Mac16Address(("00:0" + std::to_string(i + 1)).c_str()));
        } else {
            dev->GetMac()->SetExtendedAddress(Mac64Address(("00:00:00:00:00:00:00:0" + std::to_string(i + 1)).c_str()));
        }

        dev->GetPhy()->TraceConnect("TrxState", "phy" + std::to_string(i), MakeCallback(&StateChangeNotification));
        dev->GetMac()->TraceConnectWithoutContext("MacTx", MakeBoundCallback(&AsciiTrace, stream, "MacTx - dev" + std::to_string(i)));
        
        McpsDataConfirmCallback confirmCallback = MakeCallback(&DataConfirm);
        dev->GetMac()->SetMcpsDataConfirmCallback(confirmCallback);
        McpsDataIndicationCallback indicationCallback = MakeCallback(&DataIndication);
        dev->GetMac()->SetMcpsDataIndicationCallback(indicationCallback);

        Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
        mobility->SetPosition(Vector(i * 25.0, 0, 0)); // Đặt khoảng cách giữa mỗi nút là 25m
        dev->GetPhy()->SetMobility(mobility);
    }

    lrWpanHelper.EnablePcapAll("lr-wpan-all");

    // Gửi gói từ các nút 1 đến 3 đến nút 0 tại thời điểm 1 giây
    for (uint32_t i = 1; i < nodes.GetN(); ++i) {
        Ptr<Packet> packet = Create<Packet>(50);
        McpsDataRequestParams params;
        params.m_dstPanId = 0;
        params.m_srcAddrMode = extended ? EXT_ADDR : SHORT_ADDR;
        params.m_dstAddrMode = extended ? EXT_ADDR : SHORT_ADDR;
        
        if (extended) {
            params.m_dstExtAddr = Mac64Address("00:00:00:00:00:00:00:01");
        } else {
            params.m_dstAddr = Mac16Address("00:01");
        }
        
        params.m_msduHandle = i;
        params.m_txOptions = TX_OPTION_ACK;

        Simulator::ScheduleWithContext(nodes.Get(i)->GetId(), Seconds(1.0), &LrWpanMac::McpsDataRequest,
                                       devices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac(), params, packet);
    }

    // Nút 0 phản hồi tất cả các nút khác tại thời điểm 2 giây
    for (uint32_t i = 1; i < nodes.GetN(); ++i) {
        Ptr<Packet> packet = Create<Packet>(60);
        McpsDataRequestParams params;
        params.m_dstPanId = 0;
        params.m_srcAddrMode = extended ? EXT_ADDR : SHORT_ADDR;
        params.m_dstAddrMode = extended ? EXT_ADDR : SHORT_ADDR;

        if (extended) {
            params.m_dstExtAddr = Mac64Address(("00:00:00:00:00:00:00:0" + std::to_string(i + 1)).c_str());
        } else {
            params.m_dstAddr = Mac16Address(("00:0" + std::to_string(i + 1)).c_str());
        }

        params.m_msduHandle = i + 4;
        params.m_txOptions = TX_OPTION_ACK;

        Simulator::ScheduleWithContext(nodes.Get(0)->GetId(), Seconds(2.0), &LrWpanMac::McpsDataRequest,
                                       devices.Get(0)->GetObject<LrWpanNetDevice>()->GetMac(), params, packet);
    }

    Simulator::Stop(Seconds(5));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}

