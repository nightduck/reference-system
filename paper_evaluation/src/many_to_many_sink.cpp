#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rtss_evaluation/rt_system.hpp"
#include "reference_system/msg_types.hpp"

#include "rtss_evaluation/graph_system_builder.hpp"
#include "rtss_evaluation/default.hpp"

class SinkSubscriber : public rclcpp::Node
{
public:
    SinkSubscriber()
        : Node("sink_subscriber")
    {
        subscription_ = this->create_subscription<message_t>(
            "/Sink", 10, [this](const message_t::SharedPtr msg) {topic_callback(msg);});
        
        // Open the file stream in append mode
        file_stream_ = std::ofstream("timestamps.txt", std::ofstream::out);
        if (!file_stream_.is_open()) {
            throw std::runtime_error("Failed to open file");
        }
    }

    ~SinkSubscriber()
    {
        file_stream_.close();
    }

private:
    template<typename SampleTypePointer>
    void write_sample_to_file(
    SampleTypePointer & sample)
    {
        for(int i = 0; i < sample->size; i++) {
            file_stream_ << sample->stats[i].sequence_number << ":"
                << sample->stats[i].node_name.data() << ":"
                << sample->stats[i].timestamp << ",";
        }
        file_stream_ << std::endl;
    }

    void topic_callback(const message_t::SharedPtr msg)
    {
        write_sample_to_file(msg);
    }
    std::ofstream file_stream_;
    rclcpp::Subscription<message_t>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinkSubscriber>());
    rclcpp::shutdown();
    return 0;
}