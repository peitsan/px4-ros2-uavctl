#include <sms_core.h>

#include <chrono>
#include <string>
#include <thread>

class SmsClickTargetBridge : public sms::BaseNode {
public:
    SmsClickTargetBridge(
        const std::string &job_name,
        const std::string &param_file,
        const std::string &ip = "127.0.0.1",
        int port = 9094,
        const nlohmann::json &kwargs = {})
        : sms::BaseNode("SmsClickTargetBridge", job_name, param_file, ip, port, kwargs),
          target_pub_("/qr_tracker/expected_position", "geometry_msgs::Point3") {
        target_x_ = this->get_param("target_x", 0.0);
        target_y_ = this->get_param("target_y", 0.0);
        target_z_ = this->get_param("target_z", 0.0);
        target_valid_ = this->get_param("target_valid", false);
        frame_id_ = this->get_param("frame_id", std::string("base_link"));
        publish_rate_hz_ = this->get_param("publish_rate_hz", 20.0);
        target_topic_ = this->get_param("target_topic", std::string("/qr_tracker/expected_position"));
        this->params_help();
    }

    void run() {
        auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
        while (this->is_running()) {
            if (target_valid_) {
                nlohmann::json msg = sms::def_msg("geometry_msgs::Point3");
                msg["type"] = "geometry_msgs::Point3";
                msg["x"] = target_x_;
                msg["y"] = target_y_;
                msg["z"] = target_z_;
                msg["frame_id"] = frame_id_;
                msg["timestamp"] = sms::get_time_sec();
                target_pub_.publish(msg);
            }
            std::this_thread::sleep_for(period);
        }
    }

private:
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double target_z_ = 0.0;
    bool target_valid_ = false;
    std::string frame_id_ = "base_link";
    double publish_rate_hz_ = 20.0;
    std::string target_topic_ = "/qr_tracker/expected_position";

    sms::Publisher target_pub_;
};

int main(int argc, char *argv[]) {
    nlohmann::json std_params, extra_params;
    if (!sms::get_std_and_extra_args(argc, argv, std_params, extra_params)) {
        return 0;
    }

    SmsClickTargetBridge node(
        std_params["job_name"],
        std_params["config"],
        std_params["ip"],
        std_params["port"],
        extra_params);
    node.start();
    node.join();
    return 0;
}
