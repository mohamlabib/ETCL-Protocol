#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"

#include "ns3/zeal-module.h"

#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>


NS_LOG_COMPONENT_DEFINE ("ZealMinimum");

using namespace ns3;

uint16_t port = 12345;
int TxCount = 0;
int RxCount = 0;
int DropCount = 0;
double speed = 5;
Ptr<ConstantVelocityMobilityModel> cvmm;
std::vector<uint64_t> packetTraceList;
int sinkToTrace = 0;

void
StopMover()
{
   cvmm -> SetVelocity(Vector(0,0,0));//(dx,dy,dz)
}

void
InvertMover()
{
   cvmm -> SetVelocity(Vector((-1*speed),0,0)); //CHANGE THIS TO THE value of speed negative!
}

void
StartMover()
{
   cvmm -> SetVelocity(Vector(speed,0,0));
}


void
RxAppTrace (std::string context,Ptr<const Packet> p,const Address &src)
{
	if(context.find(std::to_string(sinkToTrace)) != std::string::npos)
	  {

		 //Keep track of packets id to avoid counting duplicated retransmitted packets
		 std::vector<uint64_t>::iterator it;

		  it = find (packetTraceList.begin(), packetTraceList.end(), p->GetUid());
		  if (it != packetTraceList.end())
		    {
		       //std::cout << "Element found in myvector: " << *it << '\n';
		    }
		 else
		   {
		      //std::cout << "Element not found in myvector\n";
			  RxCount = RxCount+1;
			  packetTraceList.push_back(p->GetUid());

		   }
	  }
}

void
TxAppTrace (std::string context,Ptr<const Packet> p)
{
		TxCount = TxCount+1;
		//std::cout<<"Inside TxAppTrace, packet size:" <<p->GetSize() <<"     "<< context <<std::endl;
}

void
DropTrace (std::string context, Ipv4Header const &header, Ptr< const Packet > packet, Ipv4L3Protocol::DropReason reason, Ptr<Ipv4> ipv4, uint32_t interface)
{
	/*
	 * Drop reasons Trace
	 *
	 * 1- DROP_TTL_EXPIRED 	      Packet TTL has expired.
	   2- DROP_NO_ROUTE 	      No route to host.
	   3- DROP_BAD_CHECKSUM 	  Bad checksum.
	   4- DROP_INTERFACE_DOWN 	  Interface is down so cannot send packet.
	   5- DROP_ROUTE_ERROR 	      Route error.
	   6- DROP_FRAGMENT_TIMEOUT   Fragment timeout exceeded.
	 */
    DropCount = DropCount+1;
    std::cout<<Simulator::Now().GetSeconds()<<" PACKET DROPPED " <<context <<" Reason:  "<<reason<< std::endl;
}




int
main (int argc, char *argv[])
{

   int numNodes = 120;
   double heightField = 200;
   double widthField = 400;

   double halfRound  = 80;// widthField/speed;
   int pktSize = 1024;  //packet Size in bytes
   double appDataRate = 10000; // Application Data Rate packet generation (bps)
   double mreqFactor = 0.1234;


   RngSeedManager::SetSeed(82);
   RngSeedManager::SetRun (8);

   ////////////////////// CREATE NODES ////////////////////////////////////
   NodeContainer clientNodes;
   Ptr<Node> mobileNode;

   clientNodes.Create(numNodes);
   mobileNode = CreateObject<Node>();


   std::cout<<"Nodes created\n";

   ///////////////////// CREATE DEVICES ////////////////////////////////////

   UintegerValue ctsThr = (true ? UintegerValue (100) : UintegerValue (2200));
   Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

   // A high number avoid drops in packet due to arp traffic.
   Config::SetDefault ("ns3::ArpCache::PendingQueueSize", UintegerValue (400));

   WifiHelper wifi;
   wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
   YansWifiPhyHelper wifiPhyHelper =  YansWifiPhyHelper::Default ();

   YansWifiChannelHelper wifiChannelHelper;
   wifiChannelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
   wifiChannelHelper.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(52));
   Ptr<YansWifiChannel> pchan = wifiChannelHelper.Create ();
   wifiPhyHelper.SetChannel (pchan);

   NqosWifiMacHelper wifiMacHelper = NqosWifiMacHelper::Default ();
   wifiMacHelper.SetType ("ns3::AdhocWifiMac");
   NetDeviceContainer clientDevices;
   clientDevices = wifi.Install (wifiPhyHelper, wifiMacHelper, clientNodes);
   NetDeviceContainer devices;
   devices = wifi.Install (wifiPhyHelper, wifiMacHelper, clientNodes);
   devices.Add (wifi.Install (wifiPhyHelper, wifiMacHelper, mobileNode));
   std::cout<<"Devices installed\n";

   ////////////////////////   MOBILITY  ///////////////////////////////////////////

   /////// 1- Random in a rectangle Topology

   Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
   x->SetAttribute ("Min", DoubleValue (0));
   x->SetAttribute ("Max", DoubleValue (widthField));


   Ptr<UniformRandomVariable> y = CreateObject<UniformRandomVariable>();
   y->SetAttribute ("Min", DoubleValue (0));
   y->SetAttribute ("Max", DoubleValue (heightField));



   Ptr<RandomRectanglePositionAllocator> alloc =CreateObject<RandomRectanglePositionAllocator>();
   alloc->SetX (x);
   alloc->SetY (y);
   alloc->AssignStreams(3);


  ///////// 2- Grid Topology
  /*
   Ptr<GridPositionAllocator> alloc =CreateObject<GridPositionAllocator>();
   alloc->SetMinX((m_widthField/12)/2); //initial position X for the grid
   alloc->SetMinY(0); // Initial position Y for the grid
   alloc->SetDeltaX(m_widthField/12); //12 nodes per row (space between nodes in the row)
   alloc->SetDeltaY(m_heightField/10); //10 nodes per column (space between nodes in column)
   alloc->SetLayoutType( GridPositionAllocator::COLUMN_FIRST);
   alloc->SetN(10); //1 column  10 nodes
  */

  MobilityHelper mobilityFixedNodes;
  mobilityFixedNodes.SetPositionAllocator (alloc);
  mobilityFixedNodes.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityFixedNodes.Install (clientNodes);



  MobilityHelper mobile;
  Vector posMover (0,heightField, 0); // Initial position moving node
  mobile.SetMobilityModel("ns3::ConstantVelocityMobilityModel");// no Attributes
  mobile.Install(mobileNode);

  cvmm = mobileNode->GetObject<ConstantVelocityMobilityModel> ();
  cvmm->SetPosition(posMover);


  std::cout<<"mobility set \n";


   //Mobility of Mobile Node (Sink)
   //ROUND1: SETUP ROUTES AND ZONES
   StartMover();
   Simulator::Schedule(Seconds(halfRound), &InvertMover);
   //ROUND2: DATA GATHERING
   Simulator::Schedule(Seconds(halfRound*2),&StartMover);
   Simulator::Schedule(Seconds(halfRound*3), &InvertMover);
   Simulator::Schedule(Seconds((halfRound*4)-0.1), &StopMover);

   //ROUND3: DATA GATHERING
  // Simulator::Schedule(Seconds(halfRound*4),&StartMover);
   //Simulator::Schedule(Seconds(halfRound*5), &InvertMover);
  // Simulator::Schedule(Seconds((halfRound*6)-0.1), &StopMover);

   std::cout<<"mobility set \n";

  ////////////////////////   INTERNET STACK /////////////////////////

   ZealHelper zealProtocol;
   zealProtocol.MarkSink(mobileNode);  //Marking the moving node as sink
   zealProtocol.Set("ApplicationDataRate",DoubleValue(appDataRate));//m_add trributes to m_agentfactory
   zealProtocol.Set("MreqFactor",DoubleValue(mreqFactor)); // Value of alfa

   Ipv4ListRoutingHelper listrouting;//m_This class is expected to be used in conjunction with ns3::InternetStackHelper::SetRoutingHelper
   listrouting.Add(zealProtocol, 10);//m_add protocol helper , the priority of the associated helper

   InternetStackHelper internet;     //m_aggregate IP/TCP/UDP functionality to existing Nodes.
   internet.SetRoutingHelper(listrouting);//m_Set the routing helper to use during Install. The routing helper is really an object factory which is used to create an object of type ns3::Ipv4RoutingProtocol per node. This routing object is then associated to a single ns3::Ipv4 object through its ns3::Ipv4::SetRoutingProtocol.
     
   internet.Install (clientNodes);//m_Aggregate implementations of the ns3::Ipv4, ns3::Ipv6, ns3::Udp, and ns3::Tcp classes onto the provided node.
   internet.Install (mobileNode);

   Ipv4AddressHelper ipv4; //m_A helper class to make life easier while doing simple IPv4 address assignment in scripts.
   NS_LOG_INFO ("Assign IP Addresses.");
   ipv4.SetBase ("10.1.1.0", "255.255.255.0"); //m_Set the base network number, network mask and base address.
   
   Ipv4InterfaceContainer interfaces; //m_holds a vector of std::pair of Ptr<Ipv4> and interface index. Typically ns-3 Ipv4Interfaces are installed on devices using an Ipv4 address helper.
   interfaces = ipv4.Assign (devices); //m_Assign IP addresses to the net devices specified in the container based on the current network prefix and address base.
   std::cout<<"Internet Stack installed \n";

   ///////////////////// ENERGY ///////////////////////////////////

	BasicEnergySourceHelper basicSourceHelper; //m_Creates a BasicEnergySource object.
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (3000)); //m_Sets one of the attributes of underlying EnergySource. name=Name of attribute to set,Value of the attribute. 
	 
	EnergySourceContainer energySources; //m_Holds a vector of ns3::EnergySource pointers.
	energySources = basicSourceHelper.Install (clientNodes);//m_node Pointer to the node where EnergySource will be installed.

	
	WifiRadioEnergyModelHelper radioEnergyHelper; //m_Assign WifiRadioEnergyModel to wifi devices.
	DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (clientDevices,  energySources);//m_ device=Pointer to the NetDevice to install DeviceEnergyModel,source =The EnergySource the DeviceEnergyModel will be using.

	std::cout<<"Energy SEtup complete\n";

	//////////////////// APPLICATIONS ////////////////////////////////
	uint16_t sinkPort = 9;//m_ set port to te sink.
	PacketSinkHelper sink ("ns3::UdpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (),sinkPort));
	/*m_
	 protocol(pram1)=the name of the protocol to use to receive traffic This string identifies the socket factory type used to create sockets for the applications. A typical value would be ns3::TcpSocketFactory.
      address(pram2)=the address of the sink.
     
     InetSocketAddress  Parameters==
   		ipv4(pram1)	the ipv4 address
		port(pram2) the port number
		 
		 Ipv4Address::GetAny () ==>Returns the 0.0.0.0 address.
	 */
		
	ApplicationContainer sinkApps = sink.Install (mobileNode); //m_holds a vector of ns3::Application pointers.
	//Install an ns3::PacketSinkApplication on each node of the input container configured with all the attributes set with SetAttribute. 
	//mobilenode	The node on which a PacketSinkApplication will be installed.

	sinkApps.Start (Seconds (158));//m_Arrange for all of the Applications in this container to Start() at the Time given as a parameter.
	//sinkApps.Stop (Seconds (485)); //m_Arrange for all of the Applications in this container to Stop() at the Time given as a parameter.
   sinkApps.Stop (Seconds (325));

	Ipv4Address sinkAddress = mobileNode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	/*
    GetAddress = interface	Interface number of an Ipv4 interface,addressIndex	index of Ipv4InterfaceAddress
	GetLocal() Get the local address.
	*/
	uint16_t onoffPort = 9;
	OnOffHelper onoff1 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, onoffPort));
	/*
	 Create an OnOffHelper to make it easier to work with OnOffApplications.
     protocol	the name of the protocol to use to send traffic by the applications. This string identifies the socket factory type used to create sockets for the applications. A typical value would be ns3::UdpSocketFactory.
     address	the address of the remote node to send traffic to.
	 */
	onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
	onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	onoff1.SetAttribute ("PacketSize", UintegerValue (pktSize));
	onoff1.SetAttribute ("DataRate", StringValue (std::to_string(appDataRate)));
     /*
      Helper function used to set the underlying application attributes.
      name	the name of the application attribute to set
      value	the value of the application attribute 
     */

	double deviation = 0;
	for(int i=0; i<=clientNodes.GetN()-1;  i++)
	  {
	     ApplicationContainer apps1;
		 apps1.Add(onoff1.Install (NodeList::GetNode (i)));
		 apps1.Start (Seconds (161)+Seconds(deviation));
		 apps1.Stop (Seconds (162)+Seconds(deviation));
		 deviation = deviation + 0.1;
	  }
    std::cout<<"applications installed\n";


   //////////////////  PRINT TABLES LOGS   ///////////////

    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("RoutingTables.log", std::ios::out);
	routingStream->Cleanup();
	zealProtocol.PrintRoutingTableAllAt(Seconds(halfRound*4),routingStream);

	Ptr<OutputStreamWrapper> routingStream2 = Create<OutputStreamWrapper> ("RangeTables.log", std::ios::out);
	routingStream2->Cleanup();
	zealProtocol.PrintTimeRangeTableAllAt(Seconds(halfRound*4+1),routingStream2);

	Ptr<OutputStreamWrapper> routingStream3 = Create<OutputStreamWrapper> ("AssignedTimes.txt", std::ios::out);
	routingStream3->Cleanup();
	zealProtocol.PrintGraphNodesTimes(Seconds(halfRound*4+1),routingStream3);

	Ptr<OutputStreamWrapper> routingStream4 = Create<OutputStreamWrapper> ("ZonesTest.txt", std::ios::out);
	routingStream4->Cleanup();
	zealProtocol.PrintGraphNodesZones(Seconds(halfRound*4+1),routingStream4);



		//Simulator::Stop(Seconds ((halfRound*6)+10));

	Simulator::Stop(Seconds ((halfRound*4)+10));

	Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback (&RxAppTrace));
	Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",MakeCallback (&TxAppTrace));
	Config::Connect ("/NodeList/*/$ns3::Ipv4L3Protocol/Drop",MakeCallback (&DropTrace));

	Simulator::Run ();

//////////////////mohamed energy///////////////////////////////////////
	 
     double suma = 0;
	 double rmnEnergy = 0;
	 double totalInitialEnergy = 0;
	 double eConsumed = 0;
	 double sumEConsumed = 0;
	 double maxEConsumed = 0;
	 double minEConsumed = 0;
	 double avgConsumed = 0;
	 double avgRmn = 0;
	 double minRmn = 0;
	 double maxRmn = 0;
   
    for(NodeContainer::Iterator j= clientNodes.Begin(); j!= clientNodes.End(); ++j)
		{
		  Ptr<Node> client = *j;
		  Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (energySources.Get(client->GetId()));
		  Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
		  NS_ASSERT (basicRadioModelPtr != NULL);
		  rmnEnergy = basicSourcePtr->GetRemainingEnergy ();
		  totalInitialEnergy = totalInitialEnergy+basicSourcePtr->GetInitialEnergy();
		  eConsumed = basicRadioModelPtr->GetTotalEnergyConsumption();
		  sumEConsumed = sumEConsumed+ eConsumed;

                          
						  if(maxRmn == 0)
							  maxRmn = rmnEnergy;
						  else
							{
							   if(rmnEnergy > maxRmn)
								   maxRmn = rmnEnergy;
							}



						  if(minRmn == 0)
							  minRmn = rmnEnergy;
						  else
							{
							  if(rmnEnergy < minRmn)
								 minRmn = rmnEnergy;
							}



						   // squaresum += pow(rmnEnergy,2);
						  suma += rmnEnergy;

						  std::cout << "Node: "<< basicSourcePtr->GetNode()->GetId() << "  " <<
									  " Remaining Capacity: " <<rmnEnergy  << " J (" <<
										eConsumed<<" J Consumed)"<< std::endl;
 				      	
     }
   
     avgConsumed = sumEConsumed/clientNodes.GetN();
				  //variance = (squaresum/clientNodes.GetN() - pow((suma/clientNodes.GetN()),2));
	//  avgRmn = suma/clientNodes.GetN();
     avgRmn = 3000- avgConsumed;
	
	std::cout<<"-----------------------mohamed energy------------------------------\n";		  
     std::cout <<" Average Remaining capcity: " << avgRmn << " J " << std::endl;
     std::cout <<" Average Consumtion: " << avgConsumed << " J " << std::endl;
   
	
	/////////////////end mohamed energy////////////////////////////////////////

    std::cout<<"-----------------------------------------------------\n";
	std::cout<<"TRACE TxAppTraceCount: " <<TxCount<<"\n";
	std::cout<<"TRACE RxMobileSinkTraceCount: " <<RxCount<<"\n";
	std::cout<<"TRACE DropCount: " <<DropCount<<"\n";
 	std::cout<<"-----------------------------------------------------\n";

    // clean variables
    clientNodes = NodeContainer();
    mobileNode = CreateObject<Node>();
    interfaces = Ipv4InterfaceContainer ();
    devices = NetDeviceContainer ();
    clientDevices = NetDeviceContainer();
    packetTraceList.clear();

    TxCount = 0;
    RxCount = 0;
    DropCount = 0;

    //Simulator::Destroy ();
    std::cout<<"end of simulation\n";
    std::cout<<"-----------------------------------------------------\n";
    return 0;
}
