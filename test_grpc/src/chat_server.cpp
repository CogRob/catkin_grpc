#include <memory>
#include <thread>

#include <test_grpc/grpcs/Chat.grpc.pb.h>
#include <grpcpp/grpcpp.h>
#include <ros/ros.h>

namespace test_grpc
{
class ChatServer final : public grpcs::Chat::Service
{
public:
  explicit ChatServer(const int port) : m_port(port), m_seq(0), m_thread(), m_server() {}

  ~ChatServer() {}

  grpc::Status Echo(grpc::ServerContext* ctx, const grpcs::Message* request,
                    grpcs::Message* response) override
  {
    ROS_INFO_STREAM("received " << request->body() << " from " << request->sender());

    auto now = ros::Time::now();
    auto stamp = response->mutable_header()->mutable_stamp();
    stamp->set_seconds(now.sec);
    stamp->set_nanos(now.nsec);
    response->mutable_header()->set_seq(m_seq++);
    response->set_sender(2);
    response->set_body(request->body());
    return grpc::Status::OK;
  }

  void start()
  {
    m_thread = std::thread([this]() {
      grpc::ServerBuilder builder;
      builder.SetMaxReceiveMessageSize(std::numeric_limits<int>::max());
      builder.SetMaxSendMessageSize(std::numeric_limits<int>::max());
      builder.AddListeningPort("0.0.0.0:" + std::to_string(m_port),
                               grpc::InsecureServerCredentials());
      builder.RegisterService(this);
      m_server = builder.BuildAndStart();
      ROS_INFO_STREAM("Waiting for gRPC client using port " << m_port);
      m_server->Wait();
      ROS_INFO_STREAM("Stop waiting for gRPC client using port " << m_port);
    });
  }

  void stop() {
    if (m_server) {
      const auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(1);
      m_server->Shutdown(deadline);
    }
    if (m_thread.joinable())
    {
      m_thread.join();
    }
  }

private:
  int m_port;
  int m_seq;
  std::thread m_thread;
  std::unique_ptr<grpc::Server> m_server;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chat_server");

  ros::NodeHandle pnh("~");
  const int server_port = pnh.param<int>("server_port", 50000);

  test_grpc::ChatServer server(server_port);
  server.start();
  ros::spin();
  server.stop();

  return 0;
}
