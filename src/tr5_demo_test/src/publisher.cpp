#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CSVPublisherNode : public rclcpp::Node
{
public:
    CSVPublisherNode() : Node("csv_publisher_node")
    {
        std::vector<double> cols_msg_map = {1,30,30,30,4, 3, 3, 3, 3};
        std::vector<std::string> pub_name_map = {"/time_data_topic", "/motor_pos_data_topic", "/motor_vel_data_topic", "/tau_data_topic", "/baseQuat_data_topic", "/basePos_data_topic", "/baseLinVel_data_topic", "/baseAcc_data_topic", "/baseAngVel_data_topic"};
        int num_publishers = 9; 
        publishers_.resize(num_publishers);

        for (int i = 0; i < num_publishers; i++) {
            publishers_[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                pub_name_map[i], 10);
        }
        flag_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/flag_topic", 10); 

        std::ifstream file("/home/zjy/tr5_test/mujoco_log_new.csv");
        std::string line;
        int temp = 0;

        while (std::getline(file, line)) {
            if (temp == 0) {
                temp += 1;
                std::cout << "continue" << std::endl;
                continue;
            }
            temp += 1;
            std::vector<double> row_data = parse_csv_line(line);
            std::cout << "Row data contains " << row_data.size() << " columns." << std::endl;

            int start_index = 0;
            for (int i = 0; i < num_publishers; i++) {
                int end_index = start_index + cols_msg_map[i];
                std::cout << "start " << start_index << std::endl;
                std::cout << "end " << end_index << std::endl;
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
            std::cout << "temp " << temp << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> publishers_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flag_pub_;

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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CSVPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
