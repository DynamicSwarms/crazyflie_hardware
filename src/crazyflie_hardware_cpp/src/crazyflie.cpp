#include <cstdio>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/packer/toc_packer.hpp"

#include "crtp_cpp/logic/toc_logic.hpp"


#include "crtp_cpp/logic/generic_commander_logic.hpp"
#include "crtp_cpp/logic/hl_commander_logic.hpp"
#include "crtp_cpp/logic/console_logic.hpp"
//#include "crtp_cpp/logic/hl_commander_logic.hpp"
//#include "crtp_cpp/logic/link_layer_logic.hpp"


#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "crazyflie_hardware_cpp/crtp_driver_cpp/hl_commander.hpp"


class Link : public CrtpLink
  {
  public:
    Link(std::shared_ptr<rclcpp::Node> node, int channel, std::tuple<int> address, int datarate) 
      : CrtpLink(channel, address, datarate)
      , node(node)
    {
      send_crtp_packet_client = node->create_client<crtp_interfaces::srv::CrtpPacketSend>("crazyradio/send_crtp_packet");
    }





    void send_packet_no_response(CrtpRequest request) override
    {
      auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
      req->link.channel = channel; 
      req->link.datarate = datarate;
      req->packet.port = request.packet.port;
      req->packet.channel = request.packet.channel;

      for (int i = 0; i < request.packet.data_length; i++) req->packet.data[i] = request.packet.data[i];
      req->packet.data_length = request.packet.data_length;

      send_crtp_packet_client->async_send_request(req);


      RCLCPP_WARN(node->get_logger(), "Sending no response! %d", request.packet.data_length);
    }

    std::optional<CrtpPacket> send_packet(CrtpRequest request)override
    {
      return CrtpPacket();

    }
    std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest>)override
    {
      std::vector<CrtpPacket> vec;
      return vec;
    }
  
     std::shared_ptr<rclcpp::Node> node;

    rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_client;
  };

class Commander
{
  public: 
    Commander(std::shared_ptr<rclcpp::Node> node, int channel) 
      : node(node)
      , link(node, channel, std::make_tuple(10), 250)
      , hl_commander(node, &link)
    {

    } 

  std::shared_ptr<rclcpp::Node> node;
  Link link; 
  HighLevelCommanderDriver hl_commander;
};

class CrazyflieNode : public rclcpp::Node
{
  


  public:
    CrazyflieNode(const rclcpp::NodeOptions & options)
      : Node("crazyflie", options)
    {
      this->declare_parameter("channel", 80); // Make readonly
      channel = this->get_parameter("channel").as_int();

      timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CrazyflieNode::timer_callback, this));

    }
    void init()
    {
      //!!! Makeshared not in constructor
      auto node_ptr = this->shared_from_this();
      commander = std::make_unique<Commander>(node_ptr, channel);
    }
    void timer_callback()
    {   
      // HighLevelCommanderLogic logic(&link);
      // logic.send_stop(5);
        
    }

    rclcpp::TimerBase::SharedPtr timer; 
    std::unique_ptr<Commander> commander;
    int channel; 

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello world crazyflie_hardware_cpp package\n");


  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<CrazyflieNode>(options);
  node->init();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
