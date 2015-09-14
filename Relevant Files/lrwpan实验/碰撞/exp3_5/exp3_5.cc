#include <ns3/test.h>
#include <ns3/packet.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/mobility-module.h>
#include <ns3/propagation-module.h>
#include <ns3/spectrum-module.h>
#include <ns3/mac16-address.h>
#include <ns3/mac64-address.h>
#include <ns3/log.h>
#include <ns3/node-container.h>
#include <ns3/lr-wpan-helper.h>
#include <ns3/netanim-module.h>
#include <ns3/packet-metadata.h>

NS_LOG_COMPONENT_DEFINE ("lr-wpan-collision-test");

using namespace ns3;

// This is an example TestCase.
class LrWpanCollisionTestCase : public TestCase
{
public:
  LrWpanCollisionTestCase ();
  virtual ~LrWpanCollisionTestCase ();

  void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p);

  virtual void DoRun (void);
  void setMaxPackets( int _maxPackets);//以下成员函数实现相应成员变量的重新赋值
  void setSumTimer (double _sumTimer);
  void setMinBE (uint8_t MinBE);
  void setMaxBE (uint8_t MaxBE);
  void setBacketoffs (uint8_t Backetoffs);
  void setBacketPeriod (uint8_t BacketoffPeriod);
  void setFrameRetries (uint8_t FrameRetries);

  private:
  int maxPackets;//数据包数量
  double sumTimer;//发送完maxPackets个数据包所用时间
  uint8_t m_rxPackets;//server接收到的数据包数量
  uint8_t _m_macMinBE;//以下变量对应CSMA参数
  uint8_t _m_macMaxBE;
  uint8_t _m_macMaxCSMABackoffs;
  uint8_t _m_aUnitBackoffPeriod;
  // uint8_t _m_macMaxFrameRetries;
};

void LrWpanCollisionTestCase::setMinBE (uint8_t MinBE)
{
  _m_macMinBE = MinBE;
}

void LrWpanCollisionTestCase::setMaxBE (uint8_t MaxBE)
{
  _m_macMaxBE = MaxBE;
}

void LrWpanCollisionTestCase::setBacketoffs (uint8_t Backetoffs)
{
  _m_macMaxCSMABackoffs = Backetoffs;
}

void LrWpanCollisionTestCase::setBacketPeriod (uint8_t BacketoffPeriod)
{
  _m_aUnitBackoffPeriod = BacketoffPeriod;
}

// void setFrameRetries (uint8_t FrameRetries)
// {
//   _m_macMaxFrameRetries = FrameRetries;
// }

void LrWpanCollisionTestCase::setSumTimer (double _sumTimer)
{
  sumTimer = _sumTimer;
}

void LrWpanCollisionTestCase::setMaxPackets (int _maxPackets)
{
  maxPackets = _maxPackets;
}

LrWpanCollisionTestCase::LrWpanCollisionTestCase ()
  : TestCase ("Test the 802.15.4 collision handling")
{
  m_rxPackets = 0;
  maxPackets = 0;
  sumTimer = 1;
  _m_macMinBE = 3;
  _m_macMaxBE = 5;
  _m_macMaxCSMABackoffs = 4;
  _m_aUnitBackoffPeriod = 20;
  // _m_macMaxFrameRetries = 
}

LrWpanCollisionTestCase::~LrWpanCollisionTestCase ()
{
}

void LrWpanCollisionTestCase::DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  m_rxPackets++;
}


void
LrWpanCollisionTestCase::DoRun (void)
{

  // Create 3 nodes, and a NetDevice for each one
  Ptr<Node> n0 = CreateObject <Node> ();
  Ptr<Node> n1 = CreateObject <Node> ();
  Ptr<Node> n2 = CreateObject <Node> ();
  Ptr<Node> n3 = CreateObject <Node> ();
  Ptr<Node> n4 = CreateObject <Node> ();

  NodeContainer lrNodes;
  lrNodes.Add(n0);
  lrNodes.Add(n1);
  lrNodes.Add(n2);
  lrNodes.Add(n3);
  lrNodes.Add(n4);

  Ptr<LrWpanNetDevice> dev0 = CreateObject<LrWpanNetDevice> ();
  Ptr<LrWpanNetDevice> dev1 = CreateObject<LrWpanNetDevice> ();
  Ptr<LrWpanNetDevice> dev2 = CreateObject<LrWpanNetDevice> ();
  Ptr<LrWpanNetDevice> dev3 = CreateObject<LrWpanNetDevice> ();
  Ptr<LrWpanNetDevice> dev4 = CreateObject<LrWpanNetDevice> ();

  dev0->SetAddress (Mac16Address ("00:01"));
  dev1->SetAddress (Mac16Address ("00:02"));
  dev2->SetAddress (Mac16Address ("00:03"));
  dev3->SetAddress (Mac16Address ("00:04"));
  dev4->SetAddress (Mac16Address ("00:05"));

  // Each device must be attached to the same channel
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  dev0->SetChannel (channel);
  dev1->SetChannel (channel);
  dev2->SetChannel (channel);
  dev3->SetChannel (channel);
  dev4->SetChannel (channel);

  // To complete configuration, a LrWpanNetDevice must be added to a node
  n0->AddDevice (dev0);
  n1->AddDevice (dev1);
  n2->AddDevice (dev2);
  n3->AddDevice (dev3);
  n4->AddDevice (dev4);

  Ptr<ConstantPositionMobilityModel> sender0Mobility = CreateObject<ConstantPositionMobilityModel> ();
  sender0Mobility->SetPosition (Vector (0,0,0))//设置节点0坐标为（0，0，0）
  dev0->GetPhy ()->SetMobility (sender0Mobility);
  n0->AggregateObject (sender0Mobility);

  AnimationInterface::SetConstantPosition (lrNodes.Get (0), 0, 0); 

  Ptr<ConstantPositionMobilityModel> sender1Mobility = CreateObject<ConstantPositionMobilityModel> ();
  // Configure position 10 m distance
  sender1Mobility->SetPosition (Vector (10,10,0));//设置节点1坐标为（10，10，0）
  dev1->GetPhy ()->SetMobility (sender1Mobility);
  n1->AggregateObject (sender1Mobility);

    // AnimationInterface::SetConstantPosition (lrNodes.Get (1), 10, 10); 

  Ptr<ConstantPositionMobilityModel> sender2Mobility = CreateObject<ConstantPositionMobilityModel> ();
  // Configure position 10 m distance
  sender2Mobility->SetPosition (Vector (10,-10,0));//设置节点2坐标为（10，-10，0）
  dev2->GetPhy ()->SetMobility (sender2Mobility);
  n2->AggregateObject (sender2Mobility);    

  // AnimationInterface::SetConstantPosition (lrNodes.Get (2), 10, -10); 

  Ptr<ConstantPositionMobilityModel> sender3Mobility = CreateObject<ConstantPositionMobilityModel> ();
  // Configure position 10 m distance
  sender3Mobility->SetPosition (Vector (-10,-10,0));//设置节点3坐标为（-10，-10，0）
  dev3->GetPhy ()->SetMobility (sender3Mobility);
  n3->AggregateObject (sender3Mobility);

  // AnimationInterface::SetConstantPosition (lrNodes.Get (3), 10, -10); 

  Ptr<ConstantPositionMobilityModel> sender4Mobility = CreateObject<ConstantPositionMobilityModel> ();
  // Configure position 10 m distance
  sender4Mobility->SetPosition (Vector (-10,10,0));//设置节点4坐标为（-10，10，0）
  dev4->GetPhy ()->SetMobility (sender4Mobility);
  n4->AggregateObject (sender4Mobility);

  // AnimationInterface::SetConstantPosition (lrNodes.Get (4), -10, 10); 
  //设置节点0为服务器
  dev0->GetMac ()->SetMcpsDataIndicationCallback (MakeCallback (&LrWpanCollisionTestCase::DataIndication, this));

  LrWpanHelper lrHelper;
  lrHelper.Install(lrNodes);

  // Disable first backoff
  dev0->GetCsmaCa ()->SetMacMinBE (_m_macMinBE);
  dev1->GetCsmaCa ()->SetMacMinBE (_m_macMinBE);
  dev2->GetCsmaCa ()->SetMacMinBE (_m_macMinBE);
  dev3->GetCsmaCa ()->SetMacMinBE (_m_macMinBE);
  dev4->GetCsmaCa ()->SetMacMinBE (_m_macMinBE);

  dev0->GetCsmaCa ()->SetMacMaxBE (_m_macMaxBE);
  dev1->GetCsmaCa ()->SetMacMaxBE (_m_macMaxBE);
  dev2->GetCsmaCa ()->SetMacMaxBE (_m_macMaxBE);

  dev0->GetCsmaCa ()->SetMacMaxCSMABackoffs (_m_macMaxCSMABackoffs);
  dev1->GetCsmaCa ()->SetMacMaxCSMABackoffs (_m_macMaxCSMABackoffs);
  dev2->GetCsmaCa ()->SetMacMaxCSMABackoffs (_m_macMaxCSMABackoffs);
  dev3->GetCsmaCa ()->SetMacMaxCSMABackoffs (_m_macMaxCSMABackoffs);
  dev4->GetCsmaCa ()->SetMacMaxCSMABackoffs (_m_macMaxCSMABackoffs);

  dev0->GetCsmaCa ()->SetUnitBackoffPeriod (_m_aUnitBackoffPeriod);
  dev1->GetCsmaCa ()->SetUnitBackoffPeriod (_m_aUnitBackoffPeriod);
  dev2->GetCsmaCa ()->SetUnitBackoffPeriod (_m_aUnitBackoffPeriod);
  dev3->GetCsmaCa ()->SetUnitBackoffPeriod (_m_aUnitBackoffPeriod);
  dev4->GetCsmaCa ()->SetUnitBackoffPeriod (_m_aUnitBackoffPeriod);

  // dev0->GetMac()->SetMacMaxFrameRetries(uint8_t(0));            //re
  // dev1->GetMac()->SetMacMaxFrameRetries(uint8_t(0));
  // dev2->GetMac()->SetMacMaxFrameRetries(uint8_t(0));
  // dev3->GetMac()->SetMacMaxFrameRetries(uint8_t(0));
  // dev4->GetMac()->SetMacMaxFrameRetries(uint8_t(0));

  // Ptr<Packet> p0 = Create<Packet> (20);
  // Ptr<Packet> p1 = Create<Packet> (60);

  McpsDataRequestParams params;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstPanId = 0;
  params.m_msduHandle = 0;
  params.m_dstAddr = Mac16Address ("00:01");//设置发包目标地址
  params.m_txOptions = TX_OPTION_ACK;//开启ACK


  double intervalTime = sumTimer/double(maxPackets);
  m_rxPackets = 0;
  Ptr<Packet> p;
  PacketMetadata::Enable ();
  for (int i = 0; i < maxPackets; i++){  //逐个发包
    p = Create<Packet> (100);	//设置数据包大小
    Simulator::Schedule (Seconds (i*intervalTime+0.1),
                         &LrWpanMac::McpsDataRequest,
                         dev1->GetMac (), params, p);	//节点0发包
    p = Create<Packet> (100);
    Simulator::Schedule (Seconds (i*intervalTime+0.1),
                         &LrWpanMac::McpsDataRequest,
                         dev2->GetMac (), params, p);  	//节点1发包
    p = Create<Packet> (100);
    Simulator::Schedule (Seconds (i*intervalTime+0.1),
                         &LrWpanMac::McpsDataRequest,
                         dev3->GetMac (), params, p);	//节点2发包
    p = Create<Packet> (100);
    Simulator::Schedule (Seconds (i*intervalTime+0.1),
                         &LrWpanMac::McpsDataRequest,
                         dev4->GetMac (), params, p);	//节点3发包
  }

  lrHelper.EnablePcapAll("fourToOne");

  AnimationInterface anim("exp3_5-lrWpan-collision.xml");    
  anim.EnablePacketMetadata();

  Simulator::Run ();

  std::cout << "m_rxPackets = " << int(m_rxPackets) << std::endl;

  Simulator::Destroy ();
}

// ==============================================================================
class LrWpanCollisionTestSuite : public TestSuite
{
public:
  LrWpanCollisionTestSuite ();
};

LrWpanCollisionTestSuite::LrWpanCollisionTestSuite ()
  : TestSuite ("lr-wpan-collision", UNIT)
{
  AddTestCase (new LrWpanCollisionTestCase, TestCase::QUICK);
}

static LrWpanCollisionTestSuite g_lrWpanCollisionTestSuite;


int main(int argc, char const *argv[])
{
  uint8_t MinBE = 3;
  uint8_t MaxBE = 5;
  uint8_t Backoffs = 4;
  uint8_t BackoffPeriod = 20;
  LrWpanCollisionTestCase collisionCase;//实例化碰撞模型对象
  collisionCase.setMaxPackets(100);//设置包数为100
  collisionCase.setSumTimer(1);//设置发包总时间为1s
  std::cout << "change MinBE" << std::endl;
  for (MinBE= 0; MinBE <= MaxBE; ++MinBE) //MinBE在0-MaxBE之间变换
  {
    collisionCase.setMinBE(MinBE); //设置MinBE
    std::cout << int(MinBE) << ":  " ;
    collisionCase.DoRun();//开启仿真
  }
  MinBE = 3;
  collisionCase.setMinBE(MinBE); //重置为初始状况

  std::cout << "change MaxBE" << std::endl;
  for ( MaxBE = MinBE; MaxBE <= 20; ++MaxBE) //MaxBE在MinBE-20之间变换
  {
    collisionCase.setMaxBE (MaxBE); //设置MaxBE
    std::cout << int(MaxBE) << ":  ";
    collisionCase.DoRun();	//开启仿真
  }
  MaxBE = 5;
  collisionCase.setMaxBE (MaxBE); //重置为初始状况

  std::cout << "change Backoffs" << std::endl;
  for (Backoffs = 0; Backoffs <= 20; ++Backoffs) //Backoffs在0-20之间变换
  {
    collisionCase.setBacketoffs (Backoffs);  //设置Backoffs
    std::cout << int (Backoffs) << ":  ";
    collisionCase.DoRun();//开启仿真
  }
  Backoffs = 4;
  collisionCase.setBacketoffs (Backoffs);  //重置为初始状况

  std::cout << "change BacketoffPeriod" << std::endl;
  for (BackoffPeriod = 0; BackoffPeriod <= 150; ) //BackoffPeriod在0-150间变换
  {
    collisionCase.setBacketPeriod (BackoffPeriod);//设置BackoffPeriod
    std::cout << int (BackoffPeriod) << ":  ";
    collisionCase.DoRun();//开启仿真
    BackoffPeriod += 10;
  }
  BackoffPeriod = 20;
  collisionCase.setBacketPeriod (BackoffPeriod);//重置为初始状况

  return 0;
}