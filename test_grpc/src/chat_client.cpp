#include <grpcpp/grpcpp.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <test_grpc/grpcs/Chat.grpc.pb.h>

namespace test_grpc
{
void chatClient(ros::NodeHandle& nh)
{
  ros::Publisher response_pub = nh.advertise<std_msgs::String>("response", 1, true);

  const std::string client_address =
    nh.param<std::string>("client_address", "127.0.0.1:50000");

  std::shared_ptr<grpc::Channel> channel =
      grpc::CreateChannel(client_address, grpc::InsecureChannelCredentials());
  ROS_INFO_STREAM("Connecting to " << client_address);
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(10);
  if (!channel->WaitForConnected(deadline))
  {
    ROS_ERROR_STREAM("Waiting for server timedout");
    return;
  }
  ROS_INFO_STREAM("Connected to " << client_address);

  std::unique_ptr<grpcs::Chat::Stub> stub(
      grpcs::Chat::NewStub(channel));

  grpc::ClientContext ctx;
  grpcs::Message request;
  grpcs::Message response;

  auto stamp = request.mutable_header()->mutable_stamp();
  auto now = ros::Time::now();
  stamp->set_seconds(now.sec);
  stamp->set_nanos(now.nsec);
  request.set_sender(1);
  request.set_body("foo");

  grpc::Status status = stub->Echo(&ctx, request, &response);
  if (status.ok())
  {
    ROS_INFO_STREAM("response received " << response.body() << " from " << response.sender());
    if (request.body() == response.body())
    {
      std_msgs::String msg;
      msg.data = response.body();
      // wait for test subscriber
      ros::Duration(1).sleep();
      response_pub.publish(msg);
      // wait for publish
      ros::Duration(10).sleep();
    }
    else
    {
      ROS_ERROR_STREAM("received response mismatch: " << request.body()
                                                      << " != " << response.body());
    }
  }
  else
  {
    ROS_ERROR_STREAM("Echo service returned error " << static_cast<int>(status.error_code()) << ": "
                                                    << status.error_message());
  }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chat_client");
  ros::NodeHandle pnh("~");
  test_grpc::chatClient(pnh);
  return 0;
}
