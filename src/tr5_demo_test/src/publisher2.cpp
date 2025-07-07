#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CSVPublisherNode : public rclcpp::Node
{
public:
    CSVPublisherNode() : Node("csv_publisher_node"), current_index_(0)
    {
        std::vector<double> cols_msg_map = {1,30,30,30,4, 3, 3, 3, 3};
        pub_name_map_ = {"/time_data_topic", "/motor_pos_data_topic", "/motor_vel_data_topic", "/tau_data_topic", "/baseQuat_data_topic", "/basePos_data_topic", "/baseLinVel_data_topic", "/baseAcc_data_topic", "/baseAngVel_data_topic"};
        cols_msg_map_ = cols_msg_map;

        int num_publishers = 9; 
        publishers_.resize(num_publishers);
        for (int i = 0; i < num_publishers; i++) {
            publishers_[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                pub_name_map_[i], 10);
        }
        flag_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/flag_topic", 10); 
        start_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/start_topic", 10); 

        // 读取 CSV 到 data_
        std::ifstream file("/home/ubantu/tr5_test/mujoco_log_new.csv");
        std::string line;
        int temp = 0;
        while (std::getline(file, line)) {
            if (temp == 0) { temp++; continue; }  // 跳过表头
            std::vector<double> row_data = parse_csv_line(line);
            data_.push_back(row_data);
        }

        // 设置定时器：每 10ms 调用一次 timer_callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CSVPublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (current_index_ >= data_.size()) {
            RCLCPP_INFO(this->get_logger(), "All data published.");
            timer_->cancel();  // 停止定时器
            return;
        }

        const std::vector<double>& row_data = data_[current_index_];

        int start_index = 0;
        for (size_t i = 0; i < publishers_.size(); ++i) {
            int end_index = start_index + cols_msg_map_[i];
            std::vector<double> msg_data(row_data.begin() + start_index, row_data.begin() + end_index);

            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = msg_data;
            publishers_[i]->publish(msg);

            start_index = end_index;
        }

        std::vector<double> dummy = {1, 2, 3};
        auto dummy_msg = std_msgs::msg::Float64MultiArray();
        dummy_msg.data = dummy;
        flag_pub_->publish(dummy_msg);

        current_index_++;
    }

    std::vector<double> parse_csv_line(const std::string &line)
    {
        std::vector<double> row_data;
        std::stringstream ss(line);
        std::string value;
        int column_index = 0;
        while (std::getline(ss, value, ',')) {

            if (column_index < 91 || column_index > 150) {
                row_data.push_back(std::stod(value)); 
            }
            column_index++;
        }

        return row_data;
    }

    // 成员变量
    std::vector<std::vector<double>> data_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> publishers_;
    std::vector<std::string> pub_name_map_;
    std::vector<double> cols_msg_map_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flag_pub_, start_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t current_index_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CSVPublisherNode>());
    rclcpp::shutdown();
    return 0;
}